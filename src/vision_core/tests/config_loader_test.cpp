#include "vision_core/config_loader.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

static void Reject(const std::string &yaml, const std::string &from,
                   const std::string &to) {
  std::string bad = yaml;
  const auto pos = bad.find(from); assert(pos != std::string::npos);
  bad.replace(pos, from.size(), to);
  const char *path = "invalid_algorithm.yaml";
  { std::ofstream out(path); out << bad; }
  bool rejected = false;
  try { (void)vision_core::LoadAlgorithmConfig(path); }
  catch (const std::runtime_error &) { rejected = true; }
  std::remove(path); assert(rejected);
}

static vision_core::MissionControllerConfig LoadText(const std::string &yaml) {
  const char *path = "modified_algorithm.yaml";
  { std::ofstream out(path); out << yaml; }
  try {
    auto result = vision_core::LoadAlgorithmConfig(path);
    std::remove(path);
    return result;
  } catch (...) {
    std::remove(path);
    throw;
  }
}

// Transform the fixed legacy fixture, never the operator's tuning values.
static YAML::Node GroupedFixture(const std::string &text) {
  const auto old=YAML::Load(text)["vision_algorithm"];
  YAML::Node root;
  auto a=root["vision_algorithm"];
  a["line"]["detection"]=old["line_detection"];
  a["line"]["features"]=old["line_features"];
  a["line"]["stability"]=old["line"];
  a["line"]["walk"]=old["line_p2p"];
  for(const auto* name:{"ball","hurdle","goal"}) {
    a[name]["enabled"]=old["mission"][std::string("enable_")+name];
    a[name]["control"]=YAML::Clone(old[name]);
  }
  for(const auto* name:{"ball","hurdle","goal","backboard"}) {
    auto object=std::string(name)=="backboard" ? a["goal"]["backboard"] : a[name];
    const auto prefix=std::string(name)+"_";
    for(const auto& field:old["object_targets"]) {
      const auto key=field.first.as<std::string>();
      if(key.find(prefix)==0) object["detection"][key.substr(prefix.size())]=field.second;
    }
    if(std::string(name)!="goal") object["candidate_tracking"]=old[prefix+"candidate_tracking"];
  }
  for(const auto* key:{"depth_strip_width_px","min_depth_m","max_depth_m"}) {
    const auto previous=std::string("backboard_")+key;
    a["goal"]["backboard"]["depth"][key]=old["goal"][previous];
    a["goal"]["control"].remove(previous);
  }
  a["common"]["depth_roi_size_px"]=old["object_targets"]["depth_roi_size_px"];
  a["common"]["object_association"]=old["object_association"];
  a["common"]["command"]=old["command"];
  a["common"]["mission"]=YAML::Clone(old["mission"]);
  for(const auto* name:{"ball","hurdle","goal"})
    a["common"]["mission"].remove(std::string("enable_")+name);
  return root;
}

static void CheckGroupedConfiguration(const std::string &text) {
  const auto baseline=LoadText(text);
  auto root=GroupedFixture(text);
  auto c=LoadText(YAML::Dump(root));
  assert(c.line_detection.confidence==baseline.line_detection.confidence);
  assert(c.line_features.image_center_u==baseline.line_features.image_center_u);
  assert(c.line.line_stable_window==baseline.line.line_stable_window);
  assert(c.line_p2p.ready_lead_sec==baseline.line_p2p.ready_lead_sec);
  assert(c.line_p2p.observation_start_ratio==baseline.line_p2p.observation_start_ratio);
  assert(c.ball.pickup_v2_px==baseline.ball.pickup_v2_px);
  assert(c.hurdle.contact_steps==baseline.hurdle.contact_steps);
  assert(c.goal.throw_landing_z_m==baseline.goal.throw_landing_z_m);
  assert(c.object_association.max_center_distance_norm==baseline.object_association.max_center_distance_norm);
  assert(c.command.first_action_id==baseline.command.first_action_id);
  assert(c.object_ready_mean_actions==baseline.object_ready_mean_actions);
  assert(!c.object_approach_continous && c.object_approach.lock_blocks==3);
  assert(c.object_approach.goal_empty_max_retries==3);
  root["vision_algorithm"]["common"]["mission"]["object_approach_continous"]=true;
  root["vision_algorithm"]["common"]["mission"]["object_approach"]["forward_camera"]["right_m"]=-.07;
  const auto continuous=LoadText(YAML::Dump(root));
  assert(continuous.object_approach_continous && continuous.object_approach.forward_camera.right_m==-.07);
  auto a=root["vision_algorithm"];
  // Distinct values detect accidental cross-object filter/tracker wiring.
  a["ball"]["detection"]["class_id"]=11;
  a["hurdle"]["detection"]["class_id"]=14;
  a["goal"]["detection"]["class_id"]=12;
  a["goal"]["backboard"]["detection"]["class_id"]=13;
  a["ball"]["detection"]["confidence"]=.61;
  a["hurdle"]["detection"]["confidence"]=.62;
  a["goal"]["detection"]["confidence"]=.63;
  a["goal"]["backboard"]["detection"]["confidence"]=.64;
  a["ball"]["candidate_tracking"]["max_center_distance_px"]=181;
  a["hurdle"]["candidate_tracking"]["max_center_distance_px"]=202;
  a["goal"]["backboard"]["candidate_tracking"]["max_center_distance_px"]=203;
  a["goal"]["backboard"]["depth"]["depth_strip_width_px"]=7;
  a["goal"]["backboard"]["depth"]["min_depth_m"]=.2;
  a["goal"]["backboard"]["depth"]["max_depth_m"]=2.2;
  a["common"]["depth_roi_size_px"]=7;
  a["ball"]["enabled"]=false;
  c=LoadText(YAML::Dump(root));
  assert(c.object_targets.ball_class_id==11 && c.object_targets.hurdle_class_id==14);
  assert(c.object_targets.goal_class_id==12 && c.object_targets.backboard_class_id==13);
  assert(c.object_targets.ball_confidence==.61 && c.object_targets.hurdle_confidence==.62);
  assert(c.object_targets.goal_confidence==.63 && c.object_targets.backboard_confidence==.64);
  assert(c.ball_candidate_tracking.max_center_distance_px==181);
  assert(c.hurdle_candidate_tracking.max_center_distance_px==202);
  assert(c.backboard_candidate_tracking.max_center_distance_px==203);
  assert(c.backboard_depth_strip_width_px==7 && c.backboard_min_depth_m==.2 && c.backboard_max_depth_m==2.2);
  assert(c.object_targets.depth_roi_size_px==7 && !c.enable_ball && c.enable_hurdle && c.enable_goal);
  const auto reject=[](const YAML::Node& bad) {
    bool rejected=false;
    try { (void)LoadText(YAML::Dump(bad)); }
    catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);
  };
  auto bad=YAML::Clone(root);
  bad["vision_algorithm"]["ball"]["detection"].remove("class_id");reject(bad);
  bad=YAML::Clone(root);
  bad["vision_algorithm"]["goal"]["backboard"]["depth"].remove("min_depth_m");reject(bad);
  bad=YAML::Clone(root);
  bad["vision_algorithm"]["hurdle"].remove("candidate_tracking");reject(bad);
  for (const auto *key : {"entry_recovery_max_steps", "recovery_max_steps"}) {
    bad=YAML::Clone(root);
    bad["vision_algorithm"]["hurdle"]["control"][key]=-1;reject(bad);
    bad=YAML::Clone(root);
    bad["vision_algorithm"]["hurdle"]["control"].remove(key);reject(bad);
  }
  bad=YAML::Clone(root);
  bad["vision_algorithm"]["common"].remove("depth_roi_size_px");reject(bad);
}

int main(int argc, char **argv) {
  // CTest supplies tests/fixtures/mission_scenarios.yaml, never the tuned YAML.
  // Literal values and text mutations below describe these fixed scenarios.
  assert(argc == 2);
  const auto c = vision_core::LoadAlgorithmConfig(argv[1]);
  const auto defaults = vision_core::LoadDefaultAlgorithmConfig();
  for (const auto* tracking : {&c.hurdle_candidate_tracking, &c.backboard_candidate_tracking}) {
    assert(tracking->enabled && tracking->max_center_distance_px==200);
    assert(tracking->max_size_log_ratio==.80 && tracking->missing_frame_limit==5 && tracking->max_tracks==16);
  }
  assert(c.ball.fine_motion.max_repetitions==3 && !c.ball.fine_motion.HasCalibration());
  assert(defaults.command.first_action_id != 0);
  assert(c.line_p2p.failure_min_valid_samples == 5);
  assert(c.line_p2p.observation_start_ratio == .40);
  assert(c.line_p2p.recovery_max_turns == 5);
  assert(c.line_p2p.visible_recovery_max_turns == 5);
  assert(c.line_p2p.visible_recovery_turn_yaw_deg == 15);
  assert(c.line_p2p.last_score_recovery_max_turns == 5);
  assert(c.line_p2p.last_score_recovery_turn_yaw_deg == 15);
  assert(c.goal.shoot_yaw_limit_deg == 30.0);
  assert(c.hurdle.stable_window == 10);
  assert(c.hurdle.stable_min_hits == 7);
  assert(c.hurdle.entry_recovery_max_steps == 3);
  assert(c.hurdle.recovery_max_steps == 3);
  assert(c.ball.approach_left_top_u_px==250.0);
  assert(c.ball.approach_right_top_u_px==470.0);
  assert(c.post_motion_observation_sec == 1.0);
  assert(c.object_ready_mean_actions == std::vector<int>({22, 23}));
  assert(defaults.object_ready_mean_actions == c.object_ready_mean_actions);
  assert(c.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardThree));
  assert(c.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardFour));
  assert(!c.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardLeftTwo));
  assert(c.test_stage==0 && c.test_mission==vision_core::MissionType::kNone);
  assert(c.backboard_depth_strip_width_px==5);
  assert(c.line_p2p.recovery_observation_sec == 1.0);
  assert(c.goal.recovery_center_tolerance_px == 76.8);
  assert(c.ball.recovery_max_steps == 3);
  assert(c.ball.entry_recovery_max_steps == 3);
  assert(c.ball.pickup_u1_px == 310);
  assert(c.ball.pickup_u2_px == 385);
  assert(c.ball.pickup_v1_px == 80);
  assert(c.ball.pickup_v2_px == 130);
  assert(c.ball.pickup_max_attempts == 100);
  assert(c.ball.pickup_verify_min_hits == 3);
  assert(c.goal.entry_line_retry_max_attempts == 3);
  assert(c.hurdle.contact_steps == 3);
  assert(c.hurdle.contact_inter_action_hold_sec == 0.5);
  assert(c.ball.down_search_inter_action_hold_sec == 0.0);
  assert(c.hurdle.down_search_inter_action_hold_sec == 0.0);
  assert(c.hurdle.down_one_step_v_px == 100);
  assert(c.hurdle.fine_mode_v_px == 180);
  assert(c.backboard_max_depth_m == 2.50);
  assert(c.line_features.image_center_u == 410.0);
  assert(c.camera_motion_timeout_sec == 3.0);
  assert(c.line_features.guide_min_points == 3);
  assert(c.line_features.ransac_enabled);
  assert(c.line_features.ransac_min_input_points==4);
  assert(c.line_features.ransac_residual_threshold_px==20);
  assert(c.line_features.ransac_min_sample_v_gap_px==20);
  assert(c.line_features.ransac_max_outliers==2);
  std::ifstream in(argv[1]); std::ostringstream text; text << in.rdbuf();
  CheckGroupedConfiguration(text.str());
  Reject(text.str(),"lock_blocks: 3","lock_blocks: 0");
  Reject(text.str(),"missing_frames: 3","missing_frames: -1");
  Reject(text.str(),"goal_empty_max_retries: 3","goal_empty_max_retries: -1");
  Reject(text.str(),"down_recovery_max_actions: 3","down_recovery_max_actions: 0");
  Reject(text.str(),"depth_recovery_max_steps: 3","depth_recovery_max_steps: 0");
  Reject(text.str(),"reference_v_ratio: 0.80","reference_v_ratio: .nan");
  Reject(text.str(),"object_approach_continous: false","object_approach_continous: nonsense");
  for (const auto* value : {"-0.1", "1.0", "1.1", ".nan", ".inf"})
    Reject(text.str(), "observation_start_ratio: 0.40",
           std::string("observation_start_ratio: ") + value);
  Reject(text.str(), "observation_start_ratio: 0.40", "");
  auto ratio_yaml = GroupedFixture(text.str());
  ratio_yaml["vision_algorithm"]["line"]["walk"]["observation_start_ratio"] = .20;
  assert(LoadText(YAML::Dump(ratio_yaml)).line_p2p.observation_start_ratio == .20);
  ratio_yaml["vision_algorithm"]["line"]["walk"]["observation_start_ratio"] = 0.0;
  assert(LoadText(YAML::Dump(ratio_yaml)).line_p2p.observation_start_ratio == 0.0);
  for (const auto* section : {"hurdle_candidate_tracking", "backboard_candidate_tracking"}) {
    const auto prefix=std::string(section)+":\n    enabled: true\n    max_center_distance_px: 200.0";
    Reject(text.str(), prefix, std::string(section)+":\n    enabled: true\n    max_center_distance_px: 0");
    Reject(text.str(), prefix, std::string(section)+":\n    enabled: true\n    max_center_distance_px: .nan");
    Reject(text.str(), prefix, std::string(section)+":\n    enabled: true");
    auto yaml=text.str();
    const auto start=yaml.find(prefix);assert(start!=std::string::npos);
    yaml.replace(start,prefix.size(),std::string(section)+":\n    enabled: false\n    max_center_distance_px: 123.0");
    const auto changed=LoadText(yaml);
    const auto& tracking=std::string(section)=="hurdle_candidate_tracking" ?
        changed.hurdle_candidate_tracking : changed.backboard_candidate_tracking;
    assert(!tracking.enabled && tracking.max_center_distance_px==123);
    const auto& other=std::string(section)=="hurdle_candidate_tracking" ?
        changed.backboard_candidate_tracking : changed.hurdle_candidate_tracking;
    assert(other.enabled && other.max_center_distance_px==200);
  }
  for (const auto* value : {"0","4","-1","not_a_number"})
    Reject(text.str(),"max_repetitions: 3",std::string("max_repetitions: ")+value);
  Reject(text.str(),"max_repetitions: 3","");
  for (const auto* key : {"forward_delta_u_px","forward_delta_v_px","backward_delta_u_px",
      "backward_delta_v_px","left_delta_u_px","left_delta_v_px","right_delta_u_px","right_delta_v_px"}) {
    const auto from=std::string(key)+": 0.0";
    Reject(text.str(),from,"");
    for (const auto* bad : {".nan",".inf","not_a_number"})
      Reject(text.str(),from,std::string(key)+": "+bad);
  }
  auto calibrated_yaml=text.str();
  const std::string delta_from="right_delta_u_px: 0.0";
  const auto delta_pos=calibrated_yaml.find(delta_from);assert(delta_pos!=std::string::npos);
  calibrated_yaml.replace(delta_pos,delta_from.size(),"right_delta_u_px: -50.0");
  const auto calibrated=LoadText(calibrated_yaml);
  assert(calibrated.ball.fine_motion.right_delta_px.u==-50 && calibrated.ball.fine_motion.HasCalibration());
  for (const auto* key : {"approach_left_top_u_px: 250.0", "approach_right_top_u_px: 470.0"}) {
    Reject(text.str(),key,"");
    const auto name=std::string(key).substr(0,std::string(key).find(':'));
    for (const auto* value : {"-1.0", ".nan", ".inf", "not_a_number"})
      Reject(text.str(),key,name+": "+value);
  }
  Reject(text.str(),"approach_left_top_u_px: 250.0","approach_left_top_u_px: 470.0");
  Reject(text.str(),"approach_left_top_u_px: 250.0","approach_left_top_u_px: 500.0");
  auto slanted_yaml=text.str();
  const auto top_pos=slanted_yaml.find("approach_left_top_u_px: 250.0");
  assert(top_pos!=std::string::npos);
  slanted_yaml.replace(top_pos,std::string("approach_left_top_u_px: 250.0").size(),
      "approach_left_top_u_px: 300.0");
  const auto slanted=LoadText(slanted_yaml);
  assert(slanted.ball.approach_left_top_u_px==300.0 &&
         slanted.ball.approach_right_top_u_px==470.0 &&
         slanted.ball.far_u_des_px==c.ball.far_u_des_px &&
         slanted.ball.approach_u_deadband_px==c.ball.approach_u_deadband_px);
  for (const auto* field : {"ransac_enabled: true", "ransac_min_input_points: 4",
      "ransac_residual_threshold_px: 20.0", "ransac_min_sample_v_gap_px: 20.0",
      "ransac_max_outliers: 2"})
    Reject(text.str(),field,"");
  Reject(text.str(),"ransac_enabled: true","ransac_enabled: not_a_bool");
  for (const auto* value : {"0","3","9","not_an_int"})
    Reject(text.str(),"ransac_min_input_points: 4",std::string("ransac_min_input_points: ")+value);
  for (const auto* field : {"ransac_residual_threshold_px", "ransac_min_sample_v_gap_px"})
    for (const auto* value : {"0","-1",".nan",".inf","not_a_number"})
      Reject(text.str(),std::string(field)+": 20.0",std::string(field)+": "+value);
  for (const auto* value : {"-1","3","not_an_int"})
    Reject(text.str(),"ransac_max_outliers: 2",std::string("ransac_max_outliers: ")+value);
  auto ransac_disabled=LoadText([&]() {
    auto yaml=text.str();const std::string from="ransac_enabled: true";
    yaml.replace(yaml.find(from),from.size(),"ransac_enabled: false");return yaml;
  }());
  assert(!ransac_disabled.line_features.ransac_enabled);
  auto tuned_ransac=text.str();
  for (const auto* field : {"ransac_residual_threshold_px", "ransac_min_sample_v_gap_px"}) {
    const std::string original=std::string(field)+": 20.0";
    tuned_ransac.replace(tuned_ransac.find(original),original.size(),std::string(field)+": 12.5");
  }
  const auto tuned=LoadText(tuned_ransac);
  assert(tuned.line_features.ransac_residual_threshold_px==12.5);
  assert(tuned.line_features.ransac_min_sample_v_gap_px==12.5);
  for (const auto value : {"0", "-1", "11", "not_a_number"})
    Reject(text.str(), "entry_min_hits: 7", std::string("entry_min_hits: ") + value);
  Reject(text.str(), "    entry_min_hits: 7", "    removed_entry_min_hits: 7");
  auto entry_yaml = text.str();
  const auto entry_pos = entry_yaml.find("entry_min_hits: 7");
  assert(entry_pos != std::string::npos);
  entry_yaml.replace(entry_pos, std::string("entry_min_hits: 7").size(), "entry_min_hits: 6");
  const auto entry = LoadText(entry_yaml);
  assert(entry.ball.entry_min_hits == 6 && entry.ball.stable_min_hits == 7);
  Reject(text.str(), "fine_side_two_step_margin_px: 100.0", "fine_side_two_step_margin_px: -1.0");
  Reject(text.str(), "fine_side_two_step_margin_px: 100.0", "fine_side_two_step_margin_px: .nan");
  Reject(text.str(), "fine_side_two_step_margin_px: 100.0", "removed_side_margin: 100.0");
  Reject(text.str(), "fine_side_two_step_margin_px: 100.0", "fine_side_two_step_margin_px: .inf");
  auto side_yaml=text.str();
  const auto side_pos=side_yaml.find("fine_side_two_step_margin_px: 100.0");
  assert(side_pos!=std::string::npos);
  side_yaml.replace(side_pos,std::string("fine_side_two_step_margin_px: 100.0").size(),"fine_side_two_step_margin_px: 75.0");
  assert(LoadText(side_yaml).ball.fine_side_two_step_margin_px==75.0);
  for (const auto* key : {"ball_min_box_width","ball_min_box_height",
      "goal_min_box_width","goal_min_box_height","backboard_min_box_width",
      "backboard_min_box_height","hurdle_min_box_width","hurdle_min_box_height"}) {
    const std::string original=std::string(key)+": 2.0";
    Reject(text.str(),original,"");
    Reject(text.str(),original,std::string(key)+": -1.0");
    Reject(text.str(),original,std::string(key)+": .nan");
  }
  Reject(text.str(),"down_search_max_steps: 10","down_search_max_steps: 0");
  Reject(text.str(),"down_search_max_steps: 10","down_search_max_steps: -1");
  Reject(text.str(),"down_search_max_steps: 10","");
  // Exercise the required key separately in BALL and HURDLE sections.
  for (const auto* section : {"ball:", "hurdle:"}) {
    const std::string key="down_search_inter_action_hold_sec: 0.0";
    const auto section_pos=text.str().find(section);
    assert(section_pos!=std::string::npos);
    const auto pos=text.str().find(key,section_pos);
    assert(pos!=std::string::npos);
    for (const auto* bad : {"-0.1", ".nan", ".inf", "not_a_number", ""}) {
      auto yaml=text.str();
      const std::string replacement=*bad ? std::string("down_search_inter_action_hold_sec: ")+bad : "";
      yaml.replace(pos,key.size(),replacement);
      bool rejected=false;
      try { (void)LoadText(yaml); } catch (const std::runtime_error&) { rejected=true; }
      assert(rejected);
    }
    auto yaml=text.str();
    yaml.replace(pos,key.size(),"down_search_inter_action_hold_sec: 1.25");
    const auto configured=LoadText(yaml);
    if (std::string(section)=="ball:")
      assert(configured.ball.down_search_inter_action_hold_sec==1.25 &&
             configured.hurdle.down_search_inter_action_hold_sec==0.0);
    else
      assert(configured.hurdle.down_search_inter_action_hold_sec==1.25 &&
             configured.ball.down_search_inter_action_hold_sec==0.0);
  }
  for (const auto* value : {"0","11","not_a_number"})
    Reject(text.str(),"down_search_min_hits: 7",std::string("down_search_min_hits: ")+value);
  Reject(text.str(),"down_search_min_hits: 7","");
  Reject(text.str(),"in_place_turn_score: 10.0","");
  Reject(text.str(),"in_place_turn_score: 10.0","in_place_turn_score: .nan");
  Reject(text.str(),"in_place_turn_score: 10.0","in_place_turn_score: 0.10");
  // Future opt-in only edits YAML; AUTO and tuning use the same parser.
  const auto replaced = [&text](const std::string &value) {
    std::string modified = text.str();
    const std::string original = "object_ready_mean_actions: [22, 23]";
    const auto pos = modified.find(original);
    assert(pos != std::string::npos);
    modified.replace(pos, original.size(), value);
    return modified;
  };
  auto future = LoadText(replaced("object_ready_mean_actions: [22, 23, 24, 25]"));
  assert(future.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardLeftTwo));
  assert(future.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardRightTwo));
  auto disabled = LoadText(replaced("object_ready_mean_actions: []"));
  assert(disabled.object_ready_mean_actions.empty());
  assert(!disabled.UsesObjectReadyMean(vision_core::MissionAction::kStepForwardFour));
  Reject(text.str(), "object_ready_mean_actions: [22, 23]",
         "object_ready_mean_actions: [22, 23, 22]");
  Reject(text.str(), "object_ready_mean_actions: [22, 23]",
         "object_ready_mean_actions: [10, 22]");
  Reject(text.str(), "object_ready_mean_actions: [22, 23]",
         "object_ready_mean_actions: [7]");
  Reject(text.str(), "object_ready_mean_actions: [22, 23]",
         "object_ready_mean_actions: [22, not_an_action]");
  Reject(text.str(), "object_ready_mean_actions: [22, 23]",
         "object_ready_mean_actions: 22");
  Reject(text.str(), "object_ready_mean_actions: [22, 23]", "");
  for (const auto *key : {"recovery_max_steps", "entry_recovery_max_steps"}) {
    const auto original = std::string(key) + ": 3";
    Reject(text.str(), original, std::string(key) + ": -1");
    Reject(text.str(), original, "");
  }
  Reject(text.str(), "edge_margin_px: 50.0", "edge_margin_px: -1.0");
  Reject(text.str(), "edge_margin_px: 50.0", "edge_margin_px: .nan");
  Reject(text.str(), "edge_margin_px: 50.0", "edge_margin_px: .inf");
  Reject(text.str(), "edge_margin_px: 50.0", "");
  assert(!c.goal_trial_start_goal);
  {
    auto direct_goal=text.str();
    const auto pos=direct_goal.find("trial_start_goal: false");
    assert(pos!=std::string::npos);
    direct_goal.replace(pos,std::string("trial_start_goal: false").size(),"trial_start_goal: true");
    assert(LoadText(direct_goal).goal_trial_start_goal);
  }
  Reject(text.str(), "trial_start_goal: false", "");
  Reject(text.str(), "trial_start_goal: false", "trial_start_goal: not_a_bool");
  Reject(text.str(), "post_motion_observation_sec: 1.0", "post_motion_observation_sec: 0.0");
  Reject(text.str(), "test_stage: 0", "test_stage: 4");
  Reject(text.str(), "backboard_depth_strip_width_px: 5", "backboard_depth_strip_width_px: 0");
  Reject(text.str(), "offset_gain: 0.3", "offset_gain: .nan");
  Reject(text.str(), "guide_min_points: 3", "guide_min_points: 2");
  Reject(text.str(), "guide_min_points: 3", "guide_min_points: 9");
  Reject(text.str(), "guide_min_points: 3", "guide_min_points: .nan");
  Reject(text.str(), "guide_min_points: 3", "");
  Reject(text.str(), "camera_motion_timeout_sec: 3.0",
         "camera_motion_timeout_sec: -0.1");
  Reject(text.str(), "camera_motion_timeout_sec: 3.0",
         "camera_motion_timeout_sec: .nan");
  Reject(text.str(), "camera_motion_timeout_sec: 3.0",
         "camera_motion_timeout_sec: .inf");
  Reject(text.str(), "heading_gain: 0.3", "heading_gain: -1.0");
  Reject(text.str(), "steering_deadband: 0.10", "steering_deadband: -0.1");
  Reject(text.str(), "line_stable_min_hits: 7", "line_stable_min_hits: 11");
  Reject(text.str(), "failure_min_valid_samples: 5",
         "failure_min_valid_samples: 0");
  Reject(text.str(), "pickup_u1_px: 310", "pickup_u1_px: -1.0");
  Reject(text.str(), "pickup_u1_px: 310", "pickup_u1_px: .nan");
  Reject(text.str(), "pickup_u1_px: 310", "pickup_u1_px: 386");
  Reject(text.str(), "pickup_u2_px: 385", "pickup_u2_px: 309.0");
  Reject(text.str(), "pickup_v1_px: 80", "pickup_v1_px: 131.0");
  Reject(text.str(), "pickup_v2_px: 130", "pickup_v2_px: .inf");
  Reject(text.str(), "smooth_alpha: 0.45", "smooth_alpha: .nan");
  Reject(text.str(), "approach_two_step_v_px: 240.0", "approach_two_step_v_px: .nan");
  for (const auto* value : {"-1", ".nan", ".inf", "240.0", "239.0"})
    Reject(text.str(),"approach_direct_down_v_px: 480.0",
        std::string("approach_direct_down_v_px: ")+value);
  Reject(text.str(),"approach_direct_down_v_px: 480.0","");
  auto direct_yaml=text.str();
  const auto direct_pos=direct_yaml.find("approach_direct_down_v_px: 480.0");
  assert(direct_pos!=std::string::npos);
  direct_yaml.replace(direct_pos,std::string("approach_direct_down_v_px: 480.0").size(),
      "approach_direct_down_v_px: 355.0");
  assert(LoadText(direct_yaml).ball.approach_direct_down_v_px==355.0);
  for (const auto* value : {"-1", ".nan", ".inf"})
    Reject(text.str(),"contact_inter_action_hold_sec: 0.5",
        std::string("contact_inter_action_hold_sec: ")+value);
  Reject(text.str(),"contact_inter_action_hold_sec: 0.5","");
  for (const auto* value : {"0.0", "0.8"}) {
    auto contact_yaml=text.str();
    const auto pos=contact_yaml.find("contact_inter_action_hold_sec: 0.5");
    assert(pos!=std::string::npos);
    contact_yaml.replace(pos,std::string("contact_inter_action_hold_sec: 0.5").size(),
        std::string("contact_inter_action_hold_sec: ")+value);
    assert(LoadText(contact_yaml).hurdle.contact_inter_action_hold_sec==std::stod(value));
  }
  Reject(text.str(), "pickup_max_attempts: 100", "pickup_max_attempts: 0");
  for (const auto* value : {"0", "11", ".nan"})
    Reject(text.str(), "pickup_verify_min_hits: 3",
           std::string("pickup_verify_min_hits: ")+value);
  Reject(text.str(), "pickup_verify_min_hits: 3", "");
  Reject(text.str(), "entry_line_retry_max_attempts: 3", "entry_line_retry_max_attempts: -1");
  Reject(text.str(), "entry_line_retry_max_attempts: 3", "");
  assert(LoadText([&] {
    auto modified=text.str();
    const auto position=modified.find("entry_line_retry_max_attempts: 3");
    assert(position!=std::string::npos);
    modified.replace(position, std::string("entry_line_retry_max_attempts: 3").size(),
                     "entry_line_retry_max_attempts: 0");
    return modified;
  }()).goal.entry_line_retry_max_attempts==0);
  Reject(text.str(), "upper_acquire_v_px: 470.0", "upper_acquire_v_px: -0.1");
  Reject(text.str(), "target_u_px: 380", "target_u_px: .inf");
  Reject(text.str(), "throw_landing_z_m: 0.40", "throw_landing_z_m: 0.0");
  Reject(text.str(), "throw_landing_x_m: -0.08", "throw_landing_x_m: .nan");
  Reject(text.str(), "transition_settle_sec: 0.6",
         "transition_settle_sec: -0.1");
  Reject(text.str(), "action_ack_timeout_sec: 10.0",
         "action_ack_timeout_sec: .nan");
  Reject(text.str(), "backboard_min_depth_m: 0.20",
         "backboard_min_depth_m: 6.00");
  for (const auto *key : {"visible_recovery_turn_yaw_deg",
                          "last_score_recovery_turn_yaw_deg"}) {
    const auto original = std::string(key) + ": 15";
    for (const auto *value : {"0", "181", ".nan"})
      Reject(text.str(), original, std::string(key) + ": " + value);
    Reject(text.str(), original, "");
  }
  for (const auto *key : {"visible_recovery_max_turns", "last_score_recovery_max_turns"}) {
    const auto original = std::string(key) + ": 5";
    Reject(text.str(), original, std::string(key) + ": -1");
    Reject(text.str(), original, "");
  }
  Reject(text.str(), "\n    confidence: 0.60\n    min_box_width: 1.0",
         "\n    confidence: .nan\n    min_box_width: 1.0");
  Reject(text.str(), "min_box_width: 1.0", "min_box_width: -1.0");
  Reject(text.str(), "ball_confidence: 0.60", "ball_confidence: 1.1");
  Reject(text.str(), "ball_aspect_ratio_min: 0.65", "ball_aspect_ratio_min: 1.6");
  Reject(text.str(), "ball_aspect_ratio_max: 1.50", "ball_aspect_ratio_max: .nan");
  Reject(text.str(), "hurdle_aspect_ratio_min: 2.50", "hurdle_aspect_ratio_min: -1");
  Reject(text.str(), "ball_forward_min_depth_m: 0.0", "ball_forward_min_depth_m: 3.0");
  Reject(text.str(), "ball_down_max_depth_m: 2.50", "ball_down_max_depth_m: 0");
  Reject(text.str(), "hurdle_min_depth_m: 0.30", "hurdle_min_depth_m: 2.1");
  Reject(text.str(), "depth_roi_size_px: 5", "depth_roi_size_px: 4");
  Reject(text.str(), "depth_roi_size_px: 5", "depth_roi_size_px: 33");
  Reject(text.str(), "max_center_distance_px: 80.0", "max_center_distance_px: 0");
  Reject(text.str(), "max_size_log_ratio: 0.50", "max_size_log_ratio: .inf");
  Reject(text.str(), "max_tracks: 16", "max_tracks: 0");
  Reject(text.str(), "max_tracks: 16", "max_tracks: 129");
  Reject(text.str(), "ball_depth_enabled: true", "");

  Reject(text.str(), "ball_class_id: 1", "ball_class_id: -1");
  Reject(text.str(), "ball_class_id: 1", "ball_class_id: 0");
  Reject(text.str(), "hurdle_class_id: 4", "hurdle_class_id: 0");
  Reject(text.str(), "hurdle_class_id: 4", "hurdle_class_id: 3");
  Reject(text.str(), "missing_frame_limit: 5", "missing_frame_limit: -1");
  Reject(text.str(), "center_distance_weight: 1.0",
         "center_distance_weight: -1.0");

  for (const auto& key : {std::string("transition_settle_sec: 0.6"),
                          std::string("walk_inter_action_hold_sec: 0.0"),
                          std::string("fine_motion_settle_sec: 0.0"),
                          std::string("recovery_turn_max: 3")}) {
    Reject(text.str(),key,"");
    const auto colon=key.find(':');
    Reject(text.str(),key,key.substr(0,colon)+": -1");
    Reject(text.str(),key,key.substr(0,colon)+": .nan");
  }
  auto direct = c;
  direct.line_detection.class_id = -1;
  bool direct_rejected = false;
  try { vision_core::ValidateAlgorithmConfig(direct); }
  catch (const std::runtime_error &) { direct_rejected = true; }
  assert(direct_rejected);

  auto invalid_goal = c;
  invalid_goal.goal.stable_min_hits = 0;
  bool rejected_goal = false;
  try { vision_core::ValidateAlgorithmConfig(invalid_goal); }
  catch (const std::runtime_error &) { rejected_goal = true; }
  assert(rejected_goal);

  auto relaxed_association = c;
  relaxed_association.object_association.max_center_distance_norm = 1.2;
  vision_core::ValidateAlgorithmConfig(relaxed_association);

  bool constructor_rejected = false;
  try { vision_core::MissionController invalid(direct); }
  catch (const std::runtime_error &) { constructor_rejected = true; }
  assert(constructor_rejected);
  for (int setting=0;setting<4;++setting) {
    auto invalid=c;
    if(setting==0) invalid.line_p2p.left_repetitions=0;
    if(setting==1) invalid.line_p2p.straight_repetitions=-1;
    if(setting==2) invalid.line_p2p.right_repetitions=0;
    if(setting==3) invalid.candidate_search_max_turns=-1;
    bool rejected=false;
    try { vision_core::ValidateAlgorithmConfig(invalid); }
    catch(const std::runtime_error&) { rejected=true; }
    assert(rejected);
  }
  return 0;
}
