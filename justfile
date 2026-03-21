# FRC Team 3996 — 2026 Robot
# Robot IP: 10.39.96.2 (mDNS: roborio-3996-frc.local)

robot := "lvuser@10.39.96.2"

# Pull shot tuning CSV from roboRIO
pull-shot-log:
    scp {{robot}}:/home/lvuser/shot_tuning.csv shot_tuning.csv
