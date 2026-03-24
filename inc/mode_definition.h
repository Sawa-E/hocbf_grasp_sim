namespace mc {
  enum control_mode
  {
    idle,
    DA_check,
    pos_con,
    initialposi_control,
    HOCBF_grasp,
    control_mode_size
  };

  enum record_mode
  {
    not_record,
    record_requesting,
    record_stop_requesting,
    record_pose_requesting,
    record_pose,
    recording,
    record_mode_size
  };

  enum play_mode
  {
    not_play,
    preparing,
    playing,
    play_mode_size
  };
} // namespace mc
