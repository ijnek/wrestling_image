# Manual step required

Due to software licensing issues, the 3D models for the Nao cannot be included in this repository, so it must be manually installed. You will be prompted by the installer to accept the terms and conditions.

Run the following:
```
src/nao/nao_description/install.sh
colcon build --packages-select nao_description
```
