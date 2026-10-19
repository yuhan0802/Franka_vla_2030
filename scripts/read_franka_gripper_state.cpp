// Read-only hardware probe: no Move, Grasp, or Homing commands.
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  franka::Gripper gripper(argv[1]);
  for (int i = 0; i < 5; ++i) {
    const auto state = gripper.readOnce();
    std::cout << state << std::endl;
  }
}
