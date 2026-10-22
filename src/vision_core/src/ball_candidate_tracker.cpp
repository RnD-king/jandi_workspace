#include "vision_core/ball_candidate_tracker.hpp"
#include <algorithm>
#include <cmath>
#include <tuple>

namespace vision_core {
int BallCandidateTrack::HitCount() const { return std::count(hits.begin(),hits.end(),true); }
std::optional<double> BallCandidateTrack::MeanU() const {
  double sum=0; int n=0;
  for (const auto& p:positions) if(p) {sum+=p->u; ++n;}
  return n ? std::optional<double>(sum/n) : std::nullopt;
}
const BallCandidateTrack* BallCandidateTracker::Find(std::uint64_t id) const {
  for(const auto& t:tracks_) if(t.id==id) return &t;
  return nullptr;
}
const BallCandidateTrack* BallCandidateTracker::Best() const {
  const BallCandidateTrack* best=nullptr;
  for(const auto& t:tracks_) {
    if(!best || std::make_tuple(t.HitCount(),t.detection_index.has_value(),t.last_frame) >
        std::make_tuple(best->HitCount(),best->detection_index.has_value(),best->last_frame)) best=&t;
  }
  return best; // Creation order breaks exact ties.
}
void BallCandidateTracker::Update(const std::vector<ObjectTargetCandidate>& candidates,
    const BallCandidateTrackingConfig& config,const ObjectAssociationConfig& weights,
    int window,double max_entry_v) {
  ++frame_;
  std::vector<ObjectTargetCandidate> sorted=candidates;
  std::sort(sorted.begin(),sorted.end(),[](const auto& a,const auto& b) {
    return std::make_tuple(a.target.center_px.u,a.target.center_px.v,a.target.width_px,a.target.height_px,-a.target.confidence) <
           std::make_tuple(b.target.center_px.u,b.target.center_px.v,b.target.width_px,b.target.height_px,-b.target.confidence);
  });
  struct Edge { double score,distance,size; std::size_t track,detection; };
  std::vector<Edge> edges;
  for(std::size_t i=0;i<tracks_.size();++i) {
    tracks_[i].detection_index.reset();
    for(std::size_t j=0;j<sorted.size();++j) {
      const double d=ObjectCenterDistancePixels(tracks_[i].target,sorted[j].target);
      const double s=ObjectSizeLogRatio(tracks_[i].target,sorted[j].target);
      if(d<=config.max_center_distance_px && s<=config.max_size_log_ratio)
        edges.push_back({weights.center_distance_weight*d/config.max_center_distance_px+
            weights.size_change_weight*s/config.max_size_log_ratio,d,s,i,j});
    }
  }
  std::sort(edges.begin(),edges.end(),[](const auto& a,const auto& b) {
    return std::tie(a.score,a.track,a.detection)<std::tie(b.score,b.track,b.detection);
  });
  std::vector<bool> used(sorted.size(),false),matched(tracks_.size(),false);
  for(const auto& e:edges) if(!matched[e.track] && !used[e.detection]) {
    auto& t=tracks_[e.track]; const auto& d=sorted[e.detection];
    t.target=d.target; t.detection_index=d.detection_index;
    t.center_distance_px=e.distance; t.size_log_ratio=e.size;
    t.missing_frames=0; t.last_frame=frame_;
    matched[e.track]=used[e.detection]=true;
  }
  for(std::size_t i=0;i<tracks_.size();++i) if(!matched[i]) ++tracks_[i].missing_frames;
  tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const auto& t) {
    return t.missing_frames>config.missing_frame_limit;
  }),tracks_.end());
  for(std::size_t j=0;j<sorted.size() && tracks_.size()<static_cast<std::size_t>(config.max_tracks);++j) if(!used[j]) {
    BallCandidateTrack t;
    t.id=next_id_++; t.target=sorted[j].target;
    t.detection_index=sorted[j].detection_index; t.last_frame=frame_;
    tracks_.push_back(t);
  }
  for(auto& t:tracks_) {
    const bool hit=t.detection_index && t.target.center_px.v<=max_entry_v;
    t.hits.push_back(hit);
    t.positions.push_back(hit ? std::optional<Point2>(t.target.center_px) : std::nullopt);
    while(t.hits.size()>static_cast<std::size_t>(window)) t.hits.pop_front();
    while(t.positions.size()>static_cast<std::size_t>(window)) t.positions.pop_front();
  }
}
}
