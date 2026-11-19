# Official source snapshots

| Directory | Git remote | Commit |
|---|---|---|
| openarm_control | https://github.com/enactic/openarm_control | 2f1ee56ef2bf1272329b213a1f0b0768b9d07f2b (0.4.0) |
| openarm_mujoco | https://github.com/enactic/openarm_mujoco | 06b4c8ad6d547c3faaa3bf780629ee3e1ac2b0d0 |
| dora-openarm-kinematics | https://github.com/enactic/dora-openarm-kinematics | e40dd559d07e3defdc438f459b636a87ddd39684 |
| motor-sdk | https://github.com/dmBots/motor-sdk | 0b2ede457bdbf0882e29ab9958ab8fda047b7f4a |

Preserve each repository's license. Source snapshots were downloaded for review;
only the V1 XML/assets are loaded directly at runtime. Python imports use installed
packages. In particular the model repository version and the installed resolver
package version (2.0.1) are intentionally recorded separately.

To restore a missing directory: clone its remote, fetch the recorded commit, then
checkout that commit before testing. Do not silently use a newer branch tip.
