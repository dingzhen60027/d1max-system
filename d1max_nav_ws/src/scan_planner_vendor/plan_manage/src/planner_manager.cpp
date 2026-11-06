// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <chrono>
#include <thread>
#include <bspline_opt/reference_path.hpp>
#include <bspline_opt/trajectory_timing.hpp>
#include <plan_manage/collision_reference.hpp>
#include <plan_manage/model_reference_initializer.hpp>
#include <plan_manage/stair_forward_approach.hpp>
#include <plan_manage/trajectory_collision.hpp>
#include <plan_manage/validation_cycle.hpp>

namespace scan_planner
{
  namespace
  {
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis,
                                          rclcpp::CallbackGroup::SharedPtr map_callback_group)
  {
    node_ = node;
    /* read algorithm parameters */
    const auto get_double = [node](const std::string &name, double default_value) {
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);
    reference_detour_anchor_margin_=get_double("manager.reference_detour_anchor_margin", .5);
    if (!std::isfinite(reference_detour_anchor_margin_) || reference_detour_anchor_margin_<0. ||
        reference_detour_anchor_margin_>2.) throw std::invalid_argument("invalid detour anchor margin");

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(node_,std::move(map_callback_group));

    if (!node_->has_parameter("manager.preview_body_heading_contract"))
      node_->declare_parameter<bool>("manager.preview_body_heading_contract",false);
    const bool legacy_heading_preview=
        node_->get_parameter("manager.preview_body_heading_contract").as_bool();
    // These two FSM parameters are loaded after initPlanModules(). Read their
    // immutable startup overrides as well as already-declared parameters so
    // initialization order cannot silently select the old tangent-only mode.
    const auto &overrides=node_->get_node_parameters_interface()->get_parameter_overrides();
    const auto startup_bool=[node,&overrides](const char *name,bool fallback) {
      if (node->has_parameter(name)) return node->get_parameter(name).as_bool();
      const auto override=overrides.find(name);
      return override==overrides.end()?fallback:override->second.get<bool>();
    };
    const bool execution_protocol=startup_bool("fsm.execution_protocol",false);
    // A missing writer flag is NOT a no-motion authorization. Old unmanaged
    // configurations must explicitly opt into their existing preview mode.
    const bool writer_handoff_enabled=startup_bool("fsm.writer_handoff_enabled",true);
    planning_only_geometry_=startup_bool("fsm.planning_only",false);
    rolling_candidate_connection_=execution_protocol&&!planning_only_geometry_&&
        startup_bool("fsm.local_follower_enabled",false);
    if(planning_only_geometry_&&writer_handoff_enabled)
      throw std::invalid_argument("planning-only candidate geometry cannot own writer handoff");
    preview_candidate_policy_.connector_m=get_double("manager.preview_candidate_connector_m",
        NoMotionPreviewConnectionPolicy::max_connector_m);
    preview_candidate_policy_.projection_window_m=get_double("manager.preview_candidate_projection_window_m",.5);
    if(!preview_candidate_policy_.valid())throw std::invalid_argument("invalid preview candidate geometry policy");
    preview_heading_contract_.preview_only_enabled=noMotionMeasuredHeadingContract(
        legacy_heading_preview,node_->get_parameter("grid_map.preview_only").as_bool(),
        execution_protocol,writer_handoff_enabled,
        node_->get_parameter("grid_map.use_projected_rays").as_bool(),
        grid_map_->planningRequiresObservedFree());
    preview_heading_contract_.measured_heading_enabled=measuredHeadingGeometryContract(
        legacy_heading_preview,node_->get_parameter("grid_map.preview_only").as_bool(),
        execution_protocol,writer_handoff_enabled,
        node_->get_parameter("grid_map.use_projected_rays").as_bool(),
        grid_map_->planningRequiresObservedFree());
    preview_heading_contract_.minimum_direction_speed=
        get_double("manager.preview_direction_min_speed",.02);
    preview_heading_contract_.body_extent=bodyExtent(grid_map_->planningBodyEnvelope());
    if(!node_->has_parameter("manager.body_heading_policy"))
      node_->declare_parameter<std::string>("manager.body_heading_policy","path_tangent");
    preview_heading_contract_.policy=parseBodyHeadingPolicy(
        node_->get_parameter("manager.body_heading_policy").as_string());
    if(!node_->has_parameter("manager.align_then_forward"))
      node_->declare_parameter<bool>("manager.align_then_forward",false);
    preview_heading_contract_.align_then_forward=
      node_->get_parameter("manager.align_then_forward").as_bool();
    if(preview_heading_contract_.policy==BodyHeadingPolicy::HoldHeading&&
       !measuredHeadingEnabled(preview_heading_contract_))
      throw std::invalid_argument("hold heading requires original measured body yaw");
    if (measuredHeadingEnabled(preview_heading_contract_) &&
        !validMeasuredHeadingContract(preview_heading_contract_))
      throw std::invalid_argument("invalid measured body heading geometry contract");

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMapFromEnvironment(grid_map_);

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  void SCANPlannerManager::initSolveWorker(const SCANPlannerManager &owner)
  {
    worker_only_=true;
    node_=owner.node_; // thread-safe clock/logger/immutable parameter reads only
    pp_=owner.pp_;
    preview_heading_contract_=owner.preview_heading_contract_;
    planning_only_geometry_=owner.planning_only_geometry_;
    rolling_candidate_connection_=owner.rolling_candidate_connection_;
    preview_candidate_policy_=owner.preview_candidate_policy_;
    reference_detour_anchor_margin_=owner.reference_detour_anchor_margin_;
    bspline_optimizer_rebound_=std::make_unique<BsplineOptimizer>();
    bspline_optimizer_rebound_->setParam(node_);
    bspline_optimizer_rebound_->a_star_=std::make_shared<AStar>();
    bspline_optimizer_rebound_->a_star_->initGridMapFromEnvironment(owner.grid_map_);
    bspline_optimizer_rebound_->a_star_->setEnvironment({});
  }

  void SCANPlannerManager::prepareSolveWorker(const SCANPlannerManager &owner,
                                             const GridMap::Ptr &snapshot)
  {
    if (!worker_only_ || !snapshot) throw std::logic_error("invalid solve worker snapshot");
    grid_map_=snapshot;
    // This candidate's private reader may predict full stair attitude. Include
    // the vertical lever arm in angular sampling; flat readers retain the
    // original horizontal radius. Refresh for every snapshot, not just init.
    preview_heading_contract_.body_extent=snapshot->queryBodyExtent();
    bspline_optimizer_rebound_->setEnvironment(snapshot);
    bspline_optimizer_rebound_->a_star_->setEnvironment(snapshot);
    measured_body_pose_=owner.measured_body_pose_;
    measured_body_frame_=owner.measured_body_frame_;
    measured_body_maximum_age_=owner.measured_body_maximum_age_;
    measured_body_velocity_=owner.measured_body_velocity_;
    measured_join_single_segment_=owner.measured_join_single_segment_;
    planned_splice_=owner.planned_splice_;
    local_reference_=owner.local_reference_;
    local_reference_support_=owner.local_reference_support_;
    local_reference_has_projection_connector_=owner.local_reference_has_projection_connector_;
    final_endpoint_reseed_=owner.final_endpoint_reseed_;
    // Guided mode does not use the polynomial global trajectory. Do not copy
    // uninitialized upstream storage before the first accepted local solve.
    if (owner.local_data_.traj_id_>0) local_data_=owner.local_data_;
    accepted_heading_contract_=owner.accepted_heading_contract_;
    discardPreviewCandidate();
  }

  void SCANPlannerManager::releaseSolveSnapshot() noexcept
  {
    if (!worker_only_) return; // Cleanup must never release a live owner's map.
    grid_map_.reset();
    bspline_optimizer_rebound_->setEnvironment({});
    bspline_optimizer_rebound_->a_star_->setEnvironment({});
  }

  std::optional<SCANPlannerManager::SolvedCandidate> SCANPlannerManager::takeSolvedCandidate()
  {
    if (!pending_preview_candidate_) return std::nullopt;
    SolvedCandidate result{pending_preview_candidate_->curve,pending_preview_candidate_->heading,
                           pending_preview_candidate_->context_sequence,pending_preview_candidate_->solve_body,
                           measured_join_single_segment_,pending_preview_candidate_->solve_velocity,
                           pending_preview_candidate_->splice};
    pending_preview_candidate_.reset();
    return result;
  }

  void SCANPlannerManager::importAttemptDiagnostics(const SCANPlannerManager &worker)
  {
    last_failure_phase_=worker.last_failure_phase_;
    last_timing_result_=worker.last_timing_result_;
    attempt_blocked_points_=worker.attempt_blocked_points_;
    attempt_detour_seed_=worker.attempt_detour_seed_;
  }

  bool SCANPlannerManager::commitPreviewCandidate(const SolveBudget::Ptr& budget)
  {
    if(planning_only_geometry_) {
      if(!pending_preview_candidate_)return false;
      auto candidate=takeSolvedCandidate();
      return candidate&&adoptSolvedCandidate(std::move(*candidate),budget);
    }
    return commitPreviewCandidate(); // unchanged non-formal compatibility path
  }

  bool SCANPlannerManager::adoptSolvedCandidate(SolvedCandidate candidate,
                                               const SolveBudget::Ptr &budget)
  {
    // Owner thread only. Never substitute the worker's old pose or map here.
    const auto current_lease=[&]() {
      return CandidateSourceLease{grid_map_->latestCloudStampNs(),
          grid_map_->localizationContextSequence(),grid_map_->occupancyRevision(),
          measured_body_pose_.source_stamp};
    };
    const auto sources_fresh=[&]() {
      const auto now=node_->now();
      double yaw=0.;
      return std::make_pair(
          grid_map_->integratedCloudFreshAt(now.nanoseconds()),
          measuredBodyYaw(measured_body_pose_,measured_body_frame_,now.seconds(),
                          measured_body_maximum_age_,yaw));
    };
    std::atomic_store(&solve_budget_,budget);
    if(candidate.splice) {
      // Publication is still a candidate. Recheck the immutable old prefix
      // from the latest acquired progress, never the worker's old body pose.
      if(!planned_splice_||planned_splice_->wire.incumbent_version!=candidate.splice->wire.incumbent_version||
          planned_splice_->wire.incumbent_trajectory_id!=candidate.splice->wire.incumbent_trajectory_id||
          candidate.context_sequence!=grid_map_->localizationContextSequence()||
          !sources_fresh().first||!sources_fresh().second||!budget||!budget->allowed()) {
        std::atomic_store(&solve_budget_,SolveBudget::Ptr{});last_failure_phase_="waiting_splice_incumbent";return false;
      }
      candidate.splice->measured_time=planned_splice_->measured_time;
      candidate.splice->progress_source_ns=planned_splice_->progress_source_ns;
      candidate.splice->committed_arc=planned_splice_->committed_arc;
      CurveCheckCompletion completion=CurveCheckCompletion::Uncertified;
      SpliceCheckDiagnostics splice_diagnostic;
      const bool safe=commonPrefixTailCollisionFree(candidate.curve,*candidate.splice,
          grid_map_->getResolution(),grid_map_->queryBodyExtent(),
          [&](const Eigen::Vector3d& p,double yaw){return solveAllowed(solve_budget_)?
              grid_map_->getPlanningInflateOccupancy(p,yaw):-1;},
          measured_body_pose_,measured_body_frame_,node_->now().seconds(),measured_body_maximum_age_,
          budget->remainingSeconds(),candidate.heading,&completion,&splice_diagnostic);
      std::atomic_store(&solve_budget_,SolveBudget::Ptr{});
      if(!safe||!budget->allowed()||!sources_fresh().first||!sources_fresh().second) {
        last_failure_phase_=completion==CurveCheckCompletion::Blocked?"failed_splice_prefix_collision":"waiting_splice_validation";
        RCLCPP_WARN(node_->get_logger(),
          "NativeSpliceAdoption rejected phase=%s completion=%s progress_source_ns=%ld body_source=%.9f source_delta_ms=%.3f; incumbent preserved",
          splice_diagnostic.phase,curveCheckCompletionName(completion),
          splice_diagnostic.progress_source_ns,splice_diagnostic.body_source_seconds,
          splice_diagnostic.source_delta_ms);
        return false;
      }
      updateTrajInfo(candidate.curve,node_->now());accepted_join_.reset();accepted_preview_geometry_.reset();
      accepted_heading_contract_=candidate.heading;accepted_splice_=candidate.splice;preview_curve_progress_time_=0.;
      last_failure_phase_.clear();return true;
    }
    if(planning_only_geometry_) {
      const PreviewCandidateOrigin origin{candidate.solve_body,candidate.solve_velocity,
          pp_.max_vel_,pp_.max_acc_,candidate.single_segment,preview_candidate_policy_};
      const auto accepted=certifyPreviewCandidateGeometry(candidate.curve,origin,measured_body_pose_,
          grid_map_->getResolution(),candidate.context_sequence,budget,current_lease,sources_fresh,
          [&](const PreviewCandidateGeometryEvidence& geometry) {
        return checkWholeTrajectoryCollision(candidate.curve,200000,budget->remainingSeconds(),true,
            geometry.projection.curve_time,candidate.heading,nullptr,nullptr,
            MeasuredConnectionPolicy::PreviewCandidateGeometry,origin.policy.connector_m);
      });
      std::atomic_store(&solve_budget_,SolveBudget::Ptr{});
      candidate_acceptance_.record(accepted.outcome);
      last_failure_phase_=accepted.geometry?"":candidateAcceptanceReason(accepted.outcome);
      if(!accepted.geometry)return false;
      updateTrajInfo(candidate.curve,node_->now());
      // Deliberately no Motion join evidence or fabricated acceleration. The
      // immutable original curve still begins at its acquired solve boundary.
      accepted_join_.reset();accepted_preview_geometry_=accepted.geometry;
      accepted_splice_.reset();
      accepted_heading_contract_=candidate.heading;
      preview_curve_progress_time_=accepted.geometry->projection.curve_time;
      return true;
    }
    MotionCandidateAdoptionOutcome adoption_outcome=MotionCandidateAdoptionOutcome::NotAttempted;
    const auto connection=rolling_candidate_connection_?
        MeasuredConnectionPolicy::RollingCandidateConnection:MeasuredConnectionPolicy::CandidateAdmission;
    const auto join=certifyCandidateAdoption(candidate.curve,candidate.solve_body,
        measured_body_pose_,measured_body_velocity_,grid_map_->getResolution(),
        pp_.max_vel_,pp_.max_acc_,candidate.single_segment,candidate.context_sequence,
        budget,current_lease,sources_fresh,[&](double measured_time) {
      return checkWholeTrajectoryCollision(candidate.curve,200000,
          budget->remainingSeconds(),true,measured_time,candidate.heading,nullptr,nullptr,connection);
    },&adoption_outcome,connection);
    std::atomic_store(&solve_budget_,SolveBudget::Ptr{});
    candidate_acceptance_.record(join?CandidateAcceptanceOutcome::MotionAccepted:CandidateAcceptanceOutcome::MotionRejected);
    // Checking may consume 100 ms. A fresh map at check START is not a fresh
    // map at publication, and neither source may be renewed by this operation.
    if (!join) {
      last_failure_phase_=motionCandidateAdoptionReason(adoption_outcome);
      RCLCPP_WARN(node_->get_logger(),
        "NativeCandidateAdoption rejected outcome=%s connection=%s map_age_ms=%.3f body_age_ms=%.3f solve_body_source=%.9f current_body_source=%.9f source_delta_ms=%.3f body_delta_m=%.6f current_velocity=[%.6f,%.6f,%.6f] current_map_revision=%lu context=%lu candidate_context=%lu remaining_ms=%.3f; incumbent preserved",
        motionCandidateAdoptionReason(adoption_outcome),rolling_candidate_connection_?"native_rolling":"legacy_exact_state",
        (node_->now().nanoseconds()-grid_map_->latestCloudStampNs())*1e-6,
        (node_->now().seconds()-measured_body_pose_.source_stamp)*1000.,candidate.solve_body.source_stamp,
        measured_body_pose_.source_stamp,(measured_body_pose_.source_stamp-candidate.solve_body.source_stamp)*1000.,
        (measured_body_pose_.position-candidate.solve_body.position).norm(),
        measured_body_velocity_.x(),measured_body_velocity_.y(),measured_body_velocity_.z(),
        grid_map_->occupancyRevision(),grid_map_->localizationContextSequence(),candidate.context_sequence,
        budget?budget->remainingSeconds()*1000.:0.);
      return false;
    }
    updateTrajInfo(candidate.curve,node_->now());
    accepted_join_=join;
    accepted_splice_.reset();
    accepted_preview_geometry_.reset();
    accepted_heading_contract_=candidate.heading;
    preview_curve_progress_time_=join->curve_time;
    last_failure_phase_.clear();
    return true;
  }

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj,
                                        SolveBudget::Ptr budget)
  {
    if (!budget) budget=std::make_shared<SolveBudget>();
    std::atomic_store(&solve_budget_,budget);
    bspline_optimizer_rebound_->setSolveBudget(budget);
    try {
      const bool result=reboundReplanImpl(start_pt,start_vel,start_acc,local_target_pt,
                                         local_target_vel,flag_polyInit,flag_randomPolyTraj);
      const bool allowed=budget->allowed();
      if (!allowed) {
        last_failure_phase_=budget->reason();
        discardPreviewCandidate();
      }
      bspline_optimizer_rebound_->setSolveBudget({});
      std::atomic_store(&solve_budget_,SolveBudget::Ptr{});
      // Freeze the algorithm result BEFORE optional read-only diagnostics.
      // Never add this work to a successful solve, a deadline/cancelled solve,
      // or the live owner callback. One worker emits at most once a second.
      const auto diagnostic_now=std::chrono::steady_clock::now();
      if(worker_only_&&!result&&allowed&&reference_failure_diagnostic_&&
         diagnostic_now>=next_reference_failure_diagnostic_&&budget->remainingSeconds()>=.020) {
        next_reference_failure_diagnostic_=diagnostic_now+std::chrono::seconds(1);
        const auto detail=describeReferenceSearchFailure(*grid_map_,*reference_failure_diagnostic_,
            measured_body_pose_,measured_body_frame_,node_->now().seconds(),
            measured_body_maximum_age_,budget);
        RCLCPP_WARN(node_->get_logger(),"ReferenceSearchFailure failure=%s diagnostics_ms=%.3f %s",
            last_failure_phase_.c_str(),std::chrono::duration<double,std::milli>(
                std::chrono::steady_clock::now()-diagnostic_now).count(),detail.c_str());
      }
      return result && allowed;
    } catch (...) {
      bspline_optimizer_rebound_->setSolveBudget({});
      std::atomic_store(&solve_budget_,SolveBudget::Ptr{});
      discardPreviewCandidate();
      throw;
    }
  }

  bool SCANPlannerManager::reboundReplanImpl(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {
    last_failure_phase_="failed_optimization";
    attempt_blocked_points_.clear();
    attempt_detour_seed_.clear();
    reference_failure_diagnostic_.reset();
    // Workers are reused across stair, landing and flat references. Never let
    // an earlier reference's surface parameterization survive the next solve.
    bspline_optimizer_rebound_->setSupportHeightSampler({});
    double measured_yaw=0.;
    if (!measuredBodyYaw(measured_body_pose_,measured_body_frame_,node_->now().seconds(),
                         measured_body_maximum_age_,measured_yaw) ||
        (!planned_splice_&&(start_pt-measured_body_pose_.position).norm()>grid_map_->getResolution()*.25) ||
        (planned_splice_&&(start_pt-Eigen::Vector3d(planned_splice_->wire.position.x,
          planned_splice_->wire.position.y,planned_splice_->wire.position.z)).norm()>1e-6)) {
      last_failure_phase_="waiting_body_pose";
      return false;
    }
    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2&&
        !finalEndpointShortConnectionAllowed(final_endpoint_reseed_,local_reference_,start_pt,
          local_target_pt,planned_splice_.has_value()))
    {
      last_failure_phase_="waiting_goal_reached";
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    auto t_start = std::chrono::steady_clock::now();
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    const bool guided = !local_reference_.empty();
    auto heading_contract=preview_heading_contract_;
    heading_contract.preview_only_enabled=heading_contract.preview_only_enabled && guided;
    heading_contract.measured_heading_enabled=heading_contract.measured_heading_enabled && guided;
    heading_contract.measured_yaw=planned_splice_?planned_splice_->entry_yaw:measured_yaw;
    attempt_heading_contract_=heading_contract;
    bspline_optimizer_rebound_->setPreviewHeadingContract(heading_contract);
    if (guided && local_reference_support_ && local_reference_support_->isStair() &&
        local_reference_support_->source.frame_id == grid_map_->collisionFrame()) {
      const auto model = local_reference_support_->model();
      if (model && model->valid()) {
        const auto geometry = local_reference_support_;
        bspline_optimizer_rebound_->setSupportHeightSampler(
            [geometry](const Eigen::Vector3d& body)
                -> std::optional<SupportHeightSample> {
              const auto support = geometry->model()->sampleBody(
                  body, geometry->source.body_reference_height_m);
              if (!support || std::abs(support->normal.z()) < 1e-9) return std::nullopt;
              return SupportHeightSample{support->body.z(),
                  -support->normal.head<2>() / support->normal.z()};
            });
      }
    }
    // A selected near-rest solve boundary is not an acquired state. Preserve
    // the original velocity in candidate provenance and leave current-state
    // adoption/collision checks unchanged. Future splice boundaries are exact.
    const Eigen::Vector3d acquired_start_velocity=start_vel;
    const auto forward_boundary=selectForwardMotionBoundary(start_pt,start_vel,local_reference_,
        guided&&!planning_only_geometry_&&heading_contract.align_then_forward,planned_splice_.has_value(),
        measured_yaw,local_reference_has_projection_connector_);
    if(forward_boundary.reverse_transient) {
      // An acquired reverse transient is a real boundary, not evidence that
      // the next candidate cannot join. Solve from its literal PVA; the shared
      // body-heading model and final native checks decide whether it is usable.
      // Waiting here for another state can indefinitely retain the very curve
      // whose connection was lost, without ever attempting a fresh solution.
      RCLCPP_INFO(node_->get_logger(),
          "NativeForwardBoundary reverse_transient_preserved_for_solve acquired_velocity=[%.6f,%.6f,%.6f] body_yaw=%.6f reference_forward=[%.6f,%.6f,%.6f] source=%.9f; exact PVA and native adoption checks retained",
          acquired_start_velocity.x(),acquired_start_velocity.y(),acquired_start_velocity.z(),measured_yaw,
          forward_boundary.reference_direction.x(),forward_boundary.reference_direction.y(),
          forward_boundary.reference_direction.z(),measured_body_pose_.source_stamp);
    }
    start_vel=forward_boundary.velocity;
    if(forward_boundary.regularized)RCLCPP_INFO(node_->get_logger(),
        "NativeForwardBoundary near_rest_direction_uncertain acquired_velocity=[%.6f,%.6f,%.6f] optimizer_velocity=[%.6f,%.6f,%.6f] residual_mps=%.6f join_tolerance_mps=%.6f reference_forward=[%.6f,%.6f,%.6f] source=%.9f",
        acquired_start_velocity.x(),acquired_start_velocity.y(),acquired_start_velocity.z(),
        start_vel.x(),start_vel.y(),start_vel.z(),forward_boundary.velocity_residual_mps,
        measured_candidate_join_velocity_tolerance_mps,forward_boundary.reference_direction.x(),
        forward_boundary.reference_direction.y(),forward_boundary.reference_direction.z(),measured_body_pose_.source_stamp);
    const CubicMotionBoundary motion_boundary{start_pt,start_vel,start_acc,
        local_target_pt,local_target_vel,Eigen::Vector3d::Zero()};
    if (guided && (!start_vel.allFinite() || !start_acc.allFinite() ||
        start_acc.norm()>pp_.max_acc_+1e-9)) {
      last_failure_phase_="failed_dynamics";
      RCLCPP_WARN(node_->get_logger(),
          "Initial motion invalid: speed=%.6f nominal=%.6f acceleration=%.6f/%.6f",
          start_vel.norm(),pp_.max_vel_,start_acc.norm(),pp_.max_acc_);
      ++continuous_failures_count_;
      return false;
    }
    bspline_optimizer_rebound_->reference_path_samples_.clear();
    if (guided)
    {
      std::vector<Eigen::Vector3d> collision_aware_reference;
      std::string seed_reason;
      const auto occupancy=[this,&heading_contract](const Eigen::Vector3d &p, double yaw) {
        if (!solveAllowed(solve_budget_)) return -1;
        return grid_map_->getPlanningInflateOccupancy(p,
            heading_contract.policy==BodyHeadingPolicy::HoldHeading?heading_contract.measured_yaw:yaw);
      };
      for (std::size_t i=1; i<local_reference_.size(); ++i) {
        const auto a=local_reference_[i-1], b=local_reference_[i];
        const double yaw=std::atan2(b.y()-a.y(), b.x()-a.x());
        const int count=std::min(4096, std::max(1, static_cast<int>(std::ceil(
            (b-a).norm()/(grid_map_->getResolution()*.5)))));
        for (int j=0; j<=count && attempt_blocked_points_.size()<4096; ++j) {
          if (!solveAllowed(solve_budget_)) return false;
          const Eigen::Vector3d p=a+(b-a)*(static_cast<double>(j)/count);
          if (occupancy(p, yaw)!=0) attempt_blocked_points_.push_back(p);
        }
      }
      ASTAR_RET search_result=ASTAR_RET::SEARCH_ERR;
      std::optional<ReferenceSearchFailureDiagnostic> last_search_query;
      const auto search=[this, &search_result, &start_pt, &local_target_pt, &last_search_query,
                         &heading_contract](const Eigen::Vector3d &a, const Eigen::Vector3d &b) -> DetourSearchResult {
        if (!solveAllowed(solve_budget_))
          return {};
        // Exact reference anchors are immutable. Direct and actual first/last
        // lattice edges use native collision queries, without a fictitious trip
        // to a voxel centre or rebound's movable internal-anchor mode.
        // Recover heading independently only at this solve's actual start
        // and selected goal. Intermediate reference anchors retain the proven
        // retreat/advance preference before introducing a turning detour.
        search_result=bspline_optimizer_rebound_->a_star_->AstarSearch(
            grid_map_->getResolution(), a, b, false, true,
            {(a-start_pt).norm()<=1e-9,(b-local_target_pt).norm()<=1e-9});
        last_search_query=ReferenceSearchFailureDiagnostic{a,b,start_pt,local_target_pt,
            heading_contract.policy==BodyHeadingPolicy::HoldHeading?heading_contract.measured_yaw:
              std::atan2(b.y()-a.y(),b.x()-a.x()),
            static_cast<int>(search_result),planned_splice_.has_value()};
        if (search_result==ASTAR_RET::INIT_START_OCCUPIED)
          return {DetourSearchStatus::StartEndpointOccupied,{}};
        if (search_result==ASTAR_RET::INIT_TARGET_OCCUPIED)
          return {DetourSearchStatus::TargetEndpointOccupied,{}};
        if (search_result!=ASTAR_RET::SUCCESS)
          return {};
        return {DetourSearchStatus::Success,bspline_optimizer_rebound_->a_star_->getPath()};
      };
      // Exactly one initializer consumes the original projection connector.
      // A level tread can satisfy the flat initializer: applying a second
      // shaper to its output would mislabel its first real point as projection.
      // Select from literal support ownership, never the global SDK mode.
      auto measured_guidance=measuredReferenceForwardApproach(local_reference_,
          grid_map_->getResolution(),occupancy,start_vel,heading_contract.measured_yaw,
          local_reference_has_projection_connector_,local_reference_support_.get(),grid_map_->collisionFrame());
      bool reference_ready=buildCollisionAwareReference(measured_guidance, grid_map_->getResolution(),
          occupancy, search, collision_aware_reference, seed_reason, reference_detour_anchor_margin_);
      if(!reference_ready) {
        const auto recovery=tryModelReferenceInitializer(*grid_map_,local_reference_support_,
            std::atomic_load(&solve_budget_),measured_guidance,occupancy,search,seed_reason,
            bspline_optimizer_rebound_->a_star_->lastSearchReason(),collision_aware_reference,
            seed_reason,reference_detour_anchor_margin_);
        reference_ready=recovery.accepted;
        if(recovery.attempted)RCLCPP_INFO(node_->get_logger(),
            "ModelReferenceInitializer accepted=%d result=%s; raw proposal only, restored full model guide and whole-curve checks retained",
            recovery.accepted,recovery.reason.c_str());
      }
      if (!reference_ready) {
        reference_failure_diagnostic_=last_search_query;
        last_failure_phase_=seed_reason;
        if (seed_reason=="failed_reference_search") {
          if (bspline_optimizer_rebound_->a_star_->lastSearchTimedOut())
            last_failure_phase_="solve_search_deadline_exceeded";
          else if(bspline_optimizer_rebound_->a_star_->lastSearchResourceLimited())
            last_failure_phase_="solve_search_resource_limit";
          switch(search_result) {
            case ASTAR_RET::INIT_START_OCCUPIED: last_failure_phase_="failed_reference_start_occupied";break;
            case ASTAR_RET::INIT_TARGET_OCCUPIED: last_failure_phase_="failed_reference_target_occupied";break;
            case ASTAR_RET::INIT_LATTICE_OCCUPIED: last_failure_phase_="failed_reference_lattice_occupied";break;
            case ASTAR_RET::INIT_CONNECTOR_COLLISION: last_failure_phase_="failed_reference_search_collision";break;
            case ASTAR_RET::INIT_UNOBSERVED: last_failure_phase_="waiting_observed_space";break;
            case ASTAR_RET::INIT_OUTSIDE_MAP: last_failure_phase_="failed_reference_outside_map";break;
            default: break;
          }
        }
        RCLCPP_WARN(node_->get_logger(),"Local reference rejected: %s; native_search=%s capacity=%zu bytes=%zu",
            last_failure_phase_.c_str(),bspline_optimizer_rebound_->a_star_->lastSearchReason().c_str(),
            bspline_optimizer_rebound_->a_star_->nodeCapacity(),bspline_optimizer_rebound_->a_star_->storageBytes());
        ++continuous_failures_count_;
        return false;
      }
      attempt_detour_seed_=collision_aware_reference;
      DiscreteReference local_path;
      local_path.set(collision_aware_reference);
      const auto seed = sampleReferenceSeed(local_path, pp_.max_vel_, pp_.max_acc_,
                                            start_vel.norm(), local_target_vel.norm(), pp_.ctrl_pt_dist);
      point_set = seed.samples;
      ts = seed.dt;
      start_end_derivatives = {start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero()};
      bspline_optimizer_rebound_->reference_path_samples_ = point_set;
    }
    else
    {
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      if (!solveAllowed(solve_budget_)) return false;
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              RCLCPP_ERROR(node_->get_logger(), "pseudo_arc_length is empty; aborting replan");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);
    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));
    }

    Eigen::MatrixXd ctrl_pts;
    if (guided)
      ctrl_pts=fitCubicWithFixedBoundary(point_set,ts,motion_boundary);
    else
      UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<vector<Eigen::Vector3d>> a_star_paths;
    // Stair treads can intersect the control polygon although the fitted body
    // curve is clear. Bind this candidate's actual knot interval before that
    // proxy search; keep the established flat-ground path unchanged.
    a_star_paths = grid_map_->hasQuerySupportHeightProvider() ?
        bspline_optimizer_rebound_->initControlPointsWithSpline(ctrl_pts, ts, true) :
        bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);
    if (!bspline_optimizer_rebound_->controlPointsInitialized()) {
      last_failure_phase_=bspline_optimizer_rebound_->lastFailurePhase().empty() ?
          "failed_rebound_search":bspline_optimizer_rebound_->lastFailurePhase();
      ++continuous_failures_count_;
      return false;
    }

    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    static int vis_id = 0;
    if (visualization_) {
      visualization_->displayInitPathList(point_set, 0.2, 0);
      visualization_->displayAStarList(a_star_paths, vis_id);
    }

    t_start = std::chrono::steady_clock::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!solveAllowed(solve_budget_)) return false;
    if (!flag_step_1_success)
    {
      if (!bspline_optimizer_rebound_->lastFailurePhase().empty())
        last_failure_phase_=bspline_optimizer_rebound_->lastFailurePhase();
      else if (!bspline_optimizer_rebound_->controlPointsInitialized())
        last_failure_phase_="failed_rebound_search";
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();
    t_start = std::chrono::steady_clock::now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    last_timing_result_=BoundaryTimingResult{};
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    // For a genuinely stationary start, preserve successful rebound geometry:
    // the legacy time-refinement solver also moves XYZ and can cut a detour back
    // into obstacles. A moving start still needs its original boundary solve.
    const bool stationary_guided=guided && stationaryBoundary(start_vel,start_acc);
    if (guided && !stationary_guided)
    {
      try {
        const auto refine_interior=[this](Eigen::MatrixXd &controls,double dt) {
          if (!solveAllowed(solve_budget_)) return false;
          UniformBspline reference(controls,3,dt);
          bspline_optimizer_rebound_->ref_pts_.clear();
          const int spans=controls.cols()-3;
          for (int i=0;i<=spans;++i)
            bspline_optimizer_rebound_->ref_pts_.push_back(reference.evaluateDeBoorT(i*dt));
          Eigen::MatrixXd optimized;
          const bool success=bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(
              controls,dt,optimized);
          if (success) controls=optimized;
          return success;
        };
        // SCAN's spatial-trajectory dynamics are p/v/a, not a body-yaw
        // trajectory. A low-speed velocity reversal is not an in-place turn.
        // Keep actual/input derivatives exact and do not expand every knot
        // interval to satisfy an independently inferred tangent-angle limit.
        const auto timing=refineTimingWithFixedBoundary(pos,motion_boundary,
            pp_.max_vel_,pp_.max_acc_,refine_interior,8,std::nullopt,
            [this] { return solveAllowed(solve_budget_); },true);
        last_timing_result_=timing;
        last_timing_result_.timing_search_stop_reason="native_velocity_acceleration_only";
        flag_step_2_success=timing.success;
        if (!timing.success)
          RCLCPP_WARN(node_->get_logger(),
              "Constrained moving timing rejected: %s refinements=%d v_bound=%.6f/%.6f a_bound=%.6f/%.6f",
              timing.reason.c_str(),timing.refinements,timing.speed_bound,pp_.max_vel_,
              timing.acceleration_bound,pp_.max_acc_);
      } catch (const std::exception &error) {
        RCLCPP_ERROR(node_->get_logger(),"Invalid moving boundary timing: %s",error.what());
        flag_step_2_success=false;
      }
    }
    else if (!stationary_guided && !pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (stationary_guided && flag_step_2_success)
    {
      // Only a stationary start permits geometry-preserving uniform retiming.
      // Moving starts were refitted above with exact measured derivatives.
      try
      {
        if (!solveAllowed(solve_budget_)) return false;
        last_timing_result_.time_scale=enforceDerivativeBounds(pos,pp_.max_vel_,pp_.max_acc_);
        if (!solveAllowed(solve_budget_)) return false;
        auto final_velocity=pos.getDerivative();
        last_timing_result_.speed_bound=derivativeControlBound(final_velocity);
        last_timing_result_.acceleration_bound=derivativeControlBound(final_velocity.getDerivative());
        last_timing_result_.refinements=last_timing_result_.time_scale>1.?1:0;
        last_timing_result_.success=true;
        last_timing_result_.reason="ok";
        last_timing_result_.timing_search_stop_reason="uniform_velocity_acceleration_only";
      }
      catch (const std::exception &error)
      {
        last_failure_phase_="failed_dynamics";
        RCLCPP_ERROR(node_->get_logger(), "Reject invalid native spline timing: %s", error.what());
        return false;
      }
    }

    // Production SCAN allocates spatial-trajectory time for v/a only. Exact
    // XY tangent analysis belongs to passive QA of saved final curves: it is
    // not a body-yaw trajectory and must not consume the shared solve/collision
    // deadline or silently become another acceptance gate.
    if (guided && last_timing_result_.success)
      RCLCPP_DEBUG(node_->get_logger(),
          "Native p/v/a time allocation scale=%.3f refinements=%d search_stop=%s v_bound=%.6f/%.6f a_bound=%.6f/%.6f (no body-yaw or execution claim)",
          last_timing_result_.time_scale,last_timing_result_.refinements,
          last_timing_result_.timing_search_stop_reason.c_str(),
          last_timing_result_.speed_bound,pp_.max_vel_,
          last_timing_result_.acceleration_bound,pp_.max_acc_);

    if (!solveAllowed(solve_budget_)) return false;
    if (!flag_step_2_success || !checkDynamicFeasibility(pos))
    {
      last_failure_phase_="failed_dynamics";
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }
    // Only queries on this actual final curve may classify its rejection.
    // Unknown cells explored by an earlier A* are not its terminal failure.
    grid_map_->resetCollisionDiagnostics();
    int final_query_failure=0;
    CurveCheckCompletion final_completion=CurveCheckCompletion::Uncertified;
    SpliceCheckDiagnostics splice_diagnostic;
    const bool final_clear=planned_splice_?commonPrefixTailCollisionFree(pos,*planned_splice_,
      grid_map_->getResolution(),grid_map_->queryBodyExtent(),
      [&](const Eigen::Vector3d& p,double yaw){if(!solveAllowed(solve_budget_))return -1;
        const int state=grid_map_->getPlanningInflateOccupancy(p,yaw);
        if(state!=0&&final_query_failure==0)final_query_failure=state;return state;},
      measured_body_pose_,measured_body_frame_,node_->now().seconds(),measured_body_maximum_age_,
      std::atomic_load(&solve_budget_)->remainingSeconds(),attempt_heading_contract_,&final_completion,&splice_diagnostic):
      checkWholeTrajectoryCollision(pos,200000,0.,true,0.,attempt_heading_contract_,nullptr,&final_query_failure);
    if (!final_clear) {
      last_failure_phase_=final_query_failure==2 ? "waiting_observed_space" :
          (final_query_failure==1 ? "failed_final_collision":"failed_curve_validation");
      if(planned_splice_)RCLCPP_WARN(node_->get_logger(),
        "NativeSpliceSolve rejected phase=%s completion=%s query_state=%d progress_source_ns=%ld body_source=%.9f source_delta_ms=%.3f; incumbent preserved",
        splice_diagnostic.phase,curveCheckCompletionName(final_completion),final_query_failure,
        splice_diagnostic.progress_source_ns,splice_diagnostic.body_source_seconds,splice_diagnostic.source_delta_ms);
      ++continuous_failures_count_;
      return false;
    }

    // Quality is downstream of the complete native pipeline. Keep the valid
    // baseline if additional side clearance cannot be gained in the reserved
    // budget; a comfort preference must never create a new planning failure.
    SoftClearanceWorkContext quality_context;
    if(worker_only_||rolling_candidate_connection_||planning_only_geometry_)
      quality_context.completion_reserve_s=ValidationCycle::slow_curve_budget_s+
          soft_clearance_publication_reserve_s;
    if(worker_only_||rolling_candidate_connection_||planning_only_geometry_||
        measuredHeadingEnabled(attempt_heading_contract_)) {
      const double source=measured_body_pose_.source_stamp;
      const double maximum_age=std::min(measured_body_maximum_age_,planning_only_geometry_?
          NoMotionPreviewConnectionPolicy::max_source_gap_s:RollingCurveConnectionPolicy::max_source_gap_s);
      quality_context.remaining_source=[this,source,maximum_age,last_now=source,invalid=false]() mutable {
        const double now=node_->now().seconds();
        if(invalid||!std::isfinite(source)||source<=0.||!std::isfinite(maximum_age)||maximum_age<=0.||
            !std::isfinite(now)||now<last_now) {
          invalid=true;return std::numeric_limits<double>::quiet_NaN();
        }
        last_now=now;return maximum_age-(now-source);
      };
    }
    const auto quality_source_current=[&] {
      if(!quality_context.remaining_source)return true;
      const double remaining=quality_context.remaining_source();
      return std::isfinite(remaining)&&remaining>=quality_context.completion_reserve_s;
    };
    const auto soft_clearance=bspline_optimizer_rebound_->improveTrajectoryClearance(pos,
      [&](UniformBspline& candidate,const SolveBudget::Ptr& quality_budget) {
        if(!solveAllowed(quality_budget)||!quality_source_current()||
            !checkDynamicFeasibility(candidate,quality_budget,quality_source_current))return false;
        const auto occupied=[&](const Eigen::Vector3d& p,double yaw) {
          return solveAllowed(quality_budget)&&quality_source_current()?grid_map_->getPlanningInflateOccupancy(p,yaw):-1;
        };
        if(planned_splice_)
          return commonPrefixTailCollisionFree(candidate,*planned_splice_,grid_map_->getResolution(),
              grid_map_->queryBodyExtent(),occupied,measured_body_pose_,measured_body_frame_,
              node_->now().seconds(),measured_body_maximum_age_,quality_budget->remainingSeconds(),
              attempt_heading_contract_);
        return wholeCurveCollisionFree(candidate,grid_map_->getResolution(),grid_map_->queryBodyExtent(),
            occupied,measured_body_pose_,measured_body_frame_,node_->now().seconds(),
            measured_body_maximum_age_,200000,quality_budget->remainingSeconds(),0.,attempt_heading_contract_);
      },quality_context);
    if(soft_clearance.changed) {
      auto velocity=pos.getDerivative();
      last_timing_result_.speed_bound=derivativeControlBound(velocity);
      last_timing_result_.acceleration_bound=derivativeControlBound(velocity.getDerivative());
      last_timing_result_.nominal_speed_bound=inspectInitialMotionLimits(pos,pp_.max_vel_,pp_.max_acc_).nominal_speed_bound;
    }
    if(soft_clearance.attempted||soft_clearance.reason=="source_publication_reserved"||
        soft_clearance.reason=="publication_budget_reserved")
      RCLCPP_INFO(node_->get_logger(),
          "NativeSoftClearance changed=%d reason=%s work_ms=%.3f queries=%zu sections=%zu candidates=%zu "
          "side_penalty=%.6f->%.6f min_side_m=%.6f->%.6f curvature_penalty=%.6f->%.6f max_shift_m=%.6f; baseline_preserved_on_failure",
          soft_clearance.changed,soft_clearance.reason.c_str(),soft_clearance.elapsed_ms,
          soft_clearance.queries,soft_clearance.sections,soft_clearance.candidates,
          soft_clearance.initial_penalty,soft_clearance.final_penalty,
          soft_clearance.initial_minimum_side_margin,soft_clearance.final_minimum_side_margin,
          soft_clearance.initial_curvature_penalty,soft_clearance.final_curvature_penalty,soft_clearance.maximum_shift);

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();

    // save planned results
    if (!solveAllowed(solve_budget_)) return false;
    if (worker_only_ || planning_only_geometry_ || attempt_heading_contract_.preview_only_enabled) {
      pending_preview_candidate_=PreviewCandidate{pos,attempt_heading_contract_,node_->now(),
          grid_map_->localizationContextSequence(),measured_body_pose_,acquired_start_velocity,planned_splice_};
    } else {
      updateTrajInfo(pos, node_->now());
      accepted_heading_contract_=attempt_heading_contract_;
    }

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    last_failure_phase_.clear();
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    accepted_preview_geometry_.reset();
    accepted_heading_contract_=SplineHeadingContract{};
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    accepted_join_.reset(); // No stale entry proof for emergency/legacy splines.
    accepted_preview_geometry_.reset();
    preview_curve_progress_time_=0.;
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  bool SCANPlannerManager::checkWholeTrajectoryCollision(UniformBspline &position_traj,
      std::size_t query_budget, double wall_budget_seconds, bool diagnostic, double measured_curve_time,
      const SplineHeadingContract &heading_contract, CurveCheckEvidence *evidence,
      int *query_failure_state,MeasuredConnectionPolicy connection,double preview_connector_m)
  {
    const double body_extent=grid_map_->queryBodyExtent();
    bool occupied_witness=false;
    if (query_failure_state) *query_failure_state=0;
    const bool safe=wholeCurveCollisionFree(position_traj, grid_map_->getResolution(), body_extent,
      [&](const Eigen::Vector3d &p,double yaw) {
      if (!solveAllowed(solve_budget_)) return -1;
      const int collision=grid_map_->getPlanningInflateOccupancy(p,yaw);
      if (query_failure_state && *query_failure_state==0 && collision!=0)
        *query_failure_state=collision;
      occupied_witness=occupied_witness || collision==1;
      if (collision!=0 && diagnostic)
        RCLCPP_WARN(node_->get_logger(),
          "Final curve rejected: state=%d xyz=(%.6f,%.6f,%.6f) yaw=%.6f",
          collision,p.x(),p.y(),p.z(),yaw);
      return collision;
    }, measured_body_pose_, measured_body_frame_, node_->now().seconds(), measured_body_maximum_age_,
       query_budget, wall_budget_seconds, measured_curve_time,heading_contract,0.,connection,preview_connector_m);
    double yaw=0.;
    const bool pose_fresh=measuredBodyYaw(measured_body_pose_,measured_body_frame_,node_->now().seconds(),
                                         measured_body_maximum_age_,yaw);
    if (evidence) *evidence=pose_fresh ? curveCheckEvidence(safe,occupied_witness) :
        CurveCheckEvidence::Uncertified;
    return safe && pose_fresh;
  }

  bool SCANPlannerManager::commitPreviewCandidate()
  {
    // The explicit no-motion lane needs the bounded source/geometry transaction
    // above. A legacy caller must not bypass it with this unbudgeted overload.
    if(planning_only_geometry_)return false;
    if (!pending_preview_candidate_) return true; // unchanged default/execution path
    const auto &candidate=*pending_preview_candidate_;
    double yaw=0.;
    const bool fresh=grid_map_->integratedCloudFreshAt(node_->now().nanoseconds()) &&
        candidate.context_sequence==grid_map_->localizationContextSequence() &&
        measuredBodyYaw(measured_body_pose_,measured_body_frame_,node_->now().seconds(),
                        measured_body_maximum_age_,yaw);
    if (!fresh) { pending_preview_candidate_.reset(); return false; }
    updateTrajInfo(candidate.curve,candidate.solved_at);
    accepted_heading_contract_=candidate.heading;
    pending_preview_candidate_.reset();
    return true;
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj,
      const SolveBudget::Ptr& validation_budget,const std::function<bool()>& validation_current)
  {
    if(!solveAllowed(solve_budget_)||!solveAllowed(validation_budget)||
        (validation_current&&!validation_current()))return false;
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;
    const auto recovery=inspectInitialMotionLimits(position_traj,pp_.max_vel_,pp_.max_acc_);
    if(recovery.recovering&&!recovery.valid) {
      RCLCPP_WARN(node_->get_logger(),"Initial speed recovery rejected: %s nominal_bound=%.6f recovery_end=%.6f",
          recovery.reason,recovery.nominal_speed_bound,recovery.recovery_end);
      return false;
    }

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      if (!solveAllowed(solve_budget_)||!solveAllowed(validation_budget)||
          (validation_current&&!validation_current())) return false;
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      const double current_vel_limit=recovery.recovering&&tc<recovery.recovery_end?
          recovery.peak_speed+1e-6:vel_limit;
      if (vel.norm() > current_vel_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f",
                    tc, vel.norm(), vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        RCLCPP_WARN(node_->get_logger(),
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f",
                    tc, acc.norm(), acc_limit);
        return false;
      }
    }

    return true;
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
