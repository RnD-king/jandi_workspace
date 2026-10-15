#include "p2p_motion_player.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace fs = std::filesystem;
namespace {
std::string ReadFile(const fs::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot read: " + path.string());
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::set<std::string> MappedPrograms(const fs::path& source_root) {
    const auto source = ReadFile(source_root / "src/main.cpp");
    const auto begin = source.find("std::string ProgramFileForAction(");
    const auto end = source.find("\n    }", begin);
    if (begin == std::string::npos || end == std::string::npos)
        throw std::runtime_error("cannot find executor ProgramFileForAction mapping");
    const auto mapping = source.substr(begin, end - begin);
    const std::regex returns(R"json(return\s+"([^"]+\.json)"\s*;)json");
    // A future computed path must fail visibly, rather than silently skipping
    // that action while declaring the remaining mappings valid.
    const std::regex any_return(R"(return\s+[^;]*;)");
    const std::regex empty_return(R"(return\s+\{\s*\}\s*;)");
    for (auto it = std::sregex_iterator(mapping.begin(), mapping.end(), any_return);
         it != std::sregex_iterator(); ++it) {
        const auto statement = it->str();
        if (!std::regex_match(statement, returns) && !std::regex_match(statement, empty_return))
            throw std::runtime_error("unsupported computed executor mapping: " + statement);
    }
    std::set<std::string> paths;
    for (auto it = std::sregex_iterator(mapping.begin(), mapping.end(), returns);
         it != std::sregex_iterator(); ++it) paths.insert((*it)[1].str());
    if (paths.empty()) throw std::runtime_error("no mapped motion programs found");
    return paths;
}

void ValidatePrograms(const fs::path& root, const std::set<std::string>& programs) {
    // This is the executor's production parser, with no Dxl instance, Start(),
    // Update(), encoder reads, ROS node, or motor communication.
    P2PMotionPlayer player(nullptr);
    for (const auto& relative : programs) {
        try {
            player.PrepareProgram((root / relative).string(), (root / "motions").string(), {});
        } catch (const std::exception& error) {
            throw std::runtime_error((root / relative).string() + ": " + error.what());
        }
    }
    std::cout << "PASS " << programs.size() << " mapped programs and referenced motions: "
              << root << '\n';
}

std::map<std::string, fs::path> JsonFiles(const fs::path& root) {
    std::map<std::string, fs::path> files;
    for (const auto* folder : {"motions", "missions"}) {
        const auto directory = root / folder;
        if (!fs::is_directory(directory))
            throw std::runtime_error("missing asset directory: " + directory.string());
        for (const auto& entry : fs::recursive_directory_iterator(directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".json") {
                // lexical relative paths retain the installation name even for
                // colcon symlinks whose targets are in the source directory.
                files.emplace(entry.path().lexically_relative(root).generic_string(), entry.path());
            }
        }
    }
    return files;
}

void CompareInstalled(const fs::path& source, const fs::path& installed) {
    const auto expected = JsonFiles(source);
    const auto actual = JsonFiles(installed);
    bool mismatch = false;
    for (const auto& [relative, path] : expected) {
        const auto found = actual.find(relative);
        if (found == actual.end()) {
            std::cerr << "MISSING installed JSON: " << relative << '\n';
            mismatch = true;
        } else if (ReadFile(path) != ReadFile(found->second)) {
            std::cerr << "DIFFERENT installed JSON: " << relative << '\n';
            mismatch = true;
        }
    }
    for (const auto& [relative, path] : actual) {
        if (!expected.count(relative)) {
            std::cerr << "EXTRA installed JSON: " << relative << '\n';
            mismatch = true;
        }
    }
    if (mismatch) throw std::runtime_error("source/installed JSON differ; rebuild/install the current assets");
    std::cout << "PASS " << expected.size() << " source/installed JSON files match\n";
}

void RequireRejected(const fs::path& source, const fs::path& installed) {
    try { CompareInstalled(source, installed); }
    catch (const std::runtime_error&) { return; }
    throw std::runtime_error("comparison accepted stale/missing/extra assets");
}

void SelfTest() {
    const auto root = fs::temp_directory_path() / ("jandi_asset_check_" + std::to_string(getpid()));
    struct Cleanup {
        fs::path path;
        ~Cleanup() { std::error_code ignored; fs::remove_all(path, ignored); }
    } cleanup{root};
    const auto source = root / "source", installed = root / "installed";
    for (const auto& dir : {source, installed}) {
        fs::create_directories(dir / "motions");
        fs::create_directories(dir / "missions");
        std::ofstream(dir / "motions/example.json") << "{\"version\":1}";
    }
    CompareInstalled(source, installed);
    std::ofstream(installed / "motions/example.json") << "{\"version\":2}";
    RequireRejected(source, installed);
    fs::remove(installed / "motions/example.json");
    RequireRejected(source, installed);
    fs::create_symlink(source / "motions/example.json", installed / "motions/example.json");
    CompareInstalled(source, installed); // colcon --symlink-install
    std::ofstream(installed / "missions/stale.json") << "{}";
    RequireRejected(source, installed);
    fs::remove(installed / "missions/stale.json");
    fs::create_directories(source / "src");
    std::ofstream(source / "src/main.cpp") <<
        "std::string ProgramFileForAction(int action) const\n    {\n"
        "        return \"motions/example.json\";\n    }\n";
    if (MappedPrograms(source) != std::set<std::string>{"motions/example.json"})
        throw std::runtime_error("mapping extraction did not use the current executor source");
    std::ofstream(source / "src/main.cpp") <<
        "std::string ProgramFileForAction(int action) const\n    {\n"
        "        return computed_path;\n    }\n";
    bool mapping_rejected = false;
    try { (void)MappedPrograms(source); }
    catch (const std::runtime_error&) { mapping_rejected = true; }
    if (!mapping_rejected) throw std::runtime_error("computed mapping was silently skipped");
    // Real parser must accept arbitrary valid tuning and reject bad references.
    std::ofstream motion(source / "motions/example.json");
    motion << "{\"keyframes\":[{\"duration_sec\":0.17,\"max_speed_deg_s\":73,\"positions\":{";
    bool first = true;
    for (const auto id : Dxl::MotorIds()) {
        if (!first) motion << ',';
        first = false;
        motion << '\"' << static_cast<int>(id) << "\":2048";
    }
    motion << "}}]}";
    motion.close();
    ValidatePrograms(source, {"motions/example.json"});
    std::ofstream(source / "missions/missing.json") <<
        "{\"motions\":[{\"filename\":\"absent.json\"}]}";
    bool rejected = false;
    try { ValidatePrograms(source, {"missions/missing.json"}); }
    catch (const std::exception&) { rejected = true; }
    if (!rejected) throw std::runtime_error("missing referenced motion was accepted");
    std::cout << "PASS asset checker missing/different/extra/symlink/parser scenarios\n";
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--self-test") {
            SelfTest();
            return 0;
        }
        const fs::path source(FORWARD_WALK_SOURCE_DIR);
        fs::path assets = source, installed;
        for (int i = 1; i < argc; ++i) {
            const std::string option(argv[i]);
            if ((option != "--asset-root" && option != "--installed-share") || i + 1 == argc)
                throw std::runtime_error("usage: motion_assets_check [--asset-root DIR] [--installed-share DIR]");
            if (option == "--asset-root") assets = fs::absolute(argv[++i]).lexically_normal();
            else installed = fs::absolute(argv[++i]).lexically_normal();
        }
        const auto programs = MappedPrograms(source);
        ValidatePrograms(assets, programs);
        if (!installed.empty()) {
            CompareInstalled(assets, installed);
            ValidatePrograms(installed, programs);
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
