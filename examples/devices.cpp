// mclib
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
//
// Example: every device wrapper in mclib/device, and what to do when a device
// stops answering. The program runs a conveyor, a lift, a claw and wings from
// the controller, prints every sensor to the PROS terminal twice a second, and
// names each device that is not answering on the controller screen. Change
// every port and every number to match your robot.
#include "main.h"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>

using namespace mclib::units::literals;
using mclib::device::AiVisionMode;
using mclib::device::AiVisionObject;
using mclib::device::AiVisionType;
using mclib::device::AnalogAxis;
using mclib::device::DigitalButton;

// ---------------------------------------------------------------------------
// Devices. Ports are this example's; yours differ.
// ---------------------------------------------------------------------------
// Delete the partner here and in opcontrol() if you run one controller.
mclib::device::Controller master(mclib::device::ControllerId::Master);
mclib::device::Controller partner(mclib::device::ControllerId::Partner);

// Smart ports. A negative motor port runs that motor in reverse.
mclib::device::Motor conveyor(7, mclib::device::Gearset::Blue);
mclib::device::MotorGroup lift_motors({-3, 4}, mclib::device::Gearset::Blue);
// A negative port, or true as a second argument, reverses the sensor.
mclib::device::Rotation lift_sensor(8);
mclib::device::Distance front_distance(9);
mclib::device::Optical piece_sensor(12);
// Mounted 4 in behind the turning centre. The offsets are inches: +X to the
// robot's right, +Y forward.
mclib::device::Gps gps(1, 0.0, -4.0);
mclib::device::Vision goal_camera(2);
// The second number scales every reading. To find it, spin the robot ten full
// turns by hand and divide 3600 by the rotation the IMU reported.
auto imu = std::make_shared<mclib::device::Inertial>(15, 1.0);
mclib::device::AiVision piece_camera(5);
// MockIMU is a pros::Imu with the same gain, for code that needs the PROS
// type. Despite the name it reads the sensor on its port.
MockIMU second_imu(19, 1.0);

// The wrappers keep their PROS object to themselves. For a PROS call a wrapper
// does not have, make the PROS object on the same port. toProsGearset() and
// the other toPros functions in types.hpp convert mclib's enums.
pros::Motor conveyor_pros(
    7, mclib::device::toProsGearset(mclib::device::Gearset::Blue));

// 3-wire ports on the brain. An encoder and an ultrasonic each take two ports
// next to each other, starting on A, C, E or G.
mclib::device::AdiEncoder side_tracker('A', 'B');
mclib::device::AdiUltrasonic rear_sonar('C', 'D');
// The current V2 potentiometer. Add PotentiometerType::V1 for the old one.
mclib::device::AdiPotentiometer claw_pot('E');
// LimitSwitch and Bumper are two names for AdiDigitalIn, the wrapper for any
// 3-wire switch.
mclib::device::LimitSwitch lift_bottom('F');
mclib::device::Pneumatic claw('G');
mclib::device::AdiLed status_strip('H', 30);

// 3-wire ports on an expander in smart port 10. The expander's port goes
// first.
const std::uint8_t expander_port = 10;
mclib::device::Bumper back_bumper(expander_port, 'A');
mclib::device::AdiAnalogIn light_sensor(expander_port, 'B');
// Line takes a PROS port object for an expander port. 2000 is the reading
// that counts as the line, and false says the line is lighter than the floor.
Line floor_line(pros::adi::AnalogIn({expander_port, 'C'}), 2000, false);
// The two extra arguments: extended at startup or not, and whether a high pin
// extends. The right wing's valve is plumbed the other way round.
auto left_wing = std::make_shared<mclib::device::Pneumatic>(expander_port, 'D');
auto right_wing = std::make_shared<mclib::device::Pneumatic>(expander_port,
                                                             'E', false, false);
mclib::device::PneumaticGroup wings({left_wing, right_wing});
// MockPneumatic keeps a state and writes to no port. It stands in for the
// colour sorter's piston until that is fitted.
mclib::device::MockPneumatic sorter;

const mclib::units::TrackingWheel side_wheel{
    mclib::units::Wheel::fromDiameter(2.75_in), 0.0_in, 1.0};

// ---------------------------------------------------------------------------
// Failed reads. Every wrapper has two kinds of getter.
//   Typed: position(), distance(), hue(). They return std::optional, empty
//   when the device is unplugged or the port is wrong.
//   Plain: getPositionDeg(), getDistanceMm(), getHue(). They return a number.
//   On a failed read the double ones return NaN and the integer ones return
//   PROS_ERR, which is 2147483647.
// Use the typed one. When it is empty, do not move anything that depends on
// the reading, and tell the driver. Never put a made-up number in its place.
// ---------------------------------------------------------------------------
class Faults {
 public:
  // Records whether a check passed and returns the same answer, so a read can
  // be checked and recorded in one condition. A change goes to the PROS
  // terminal.
  bool check(const std::string& name, bool passed) {
    const bool was_failing = m_failing.contains(name);
    if (passed && was_failing) {
      m_failing.erase(name);
    } else if (!passed && !was_failing) {
      m_failing.insert(name);
    } else {
      return passed;
    }
    std::printf("%s %s\n", name.c_str(), passed ? "is back" : "FAILED");
    m_screen_is_stale = true;
    return passed;
  }

  bool any() const { return !m_failing.empty(); }

  // Line 0 of the controller: how many checks are failing, and one of them.
  // The controller takes one screen or rumble write about every 50 ms and
  // refuses the next until then, so this writes on a change only and keeps
  // trying until a write is accepted.
  void show(mclib::device::Controller& controller) {
    if (!m_screen_is_stale) {
      return;
    }
    const bool accepted =
        m_failing.empty()
            ? controller.clearLine(0)
            : controller.print(0, 0, "%d DOWN %-8s",
                               static_cast<int>(m_failing.size()),
                               m_failing.begin()->c_str());
    m_screen_is_stale = !accepted;
  }

 private:
  std::set<std::string> m_failing;
  bool m_screen_is_stale = false;
};

Faults faults;

// Passes a typed reading through and records whether it was there.
template <typename Reading>
std::optional<Reading> checked(const char* device,
                               std::optional<Reading> reading) {
  faults.check(device, reading.has_value());
  return reading;
}

// A setter that returns std::int32_t returns PROS_ERR when the device did not
// take it. PROS leaves the reason in errno: ENODEV when the port has no such
// device on it, ENXIO when the port number is out of range.
void reportIfRefused(const char* step, std::int32_t result) {
  if (result == PROS_ERR) {
    std::printf("%s failed: %s\n", step, std::strerror(errno));
  }
}

// True on the passes that print readings to the PROS terminal.
bool report_due = false;

// ---------------------------------------------------------------------------
// Controller. Sticks and buttons are read where they are used, further down.
// ---------------------------------------------------------------------------
// The screen is 3 lines of 15 characters. The text is padded to a full line
// so that nothing of the message before it is left showing.
void showOnController(std::uint8_t line, std::string text) {
  text.resize(15, ' ');
  if (!master.setText(line, 0, text)) {
    std::printf("the controller did not take \"%s\"\n", text.c_str());
  }
}

void showBanner() {
  // Two writes in a row need 50 ms between them.
  if (!master.clear()) {
    std::printf("the controller screen did not clear\n");
  }
  pros::delay(50);
  showOnController(1, "DEVICE CHECK");
  pros::delay(50);
  showOnController(2, "A=CLAW UP=WINGS");
}

void checkController(mclib::device::Controller& controller) {
  const bool is_master =
      controller.id() == mclib::device::ControllerId::Master;
  const char* name = is_master ? "MASTER" : "PARTNER";

  // A disconnected controller reads centred sticks and no buttons, so
  // everything this program drives from it stops.
  if (!faults.check(name, controller.isConnected()) || !report_due) {
    return;
  }
  const std::optional<std::int32_t> percent = controller.batteryCapacity();
  const std::optional<std::int32_t> level = controller.batteryLevel();
  if (percent && level) {
    std::printf("%s battery %d%%, level %d\n", name,
                static_cast<int>(*percent), static_cast<int>(*level));
  }
}

// ---------------------------------------------------------------------------
// Motor: the conveyor.
// ---------------------------------------------------------------------------
void setUpConveyor() {
  conveyor.setBrakeMode(mclib::device::BrakeMode::Coast);
  conveyor.tarePosition();
}

bool conveyorIsJammed() {
  // Typed readings compare against typed limits. The plain getCurrentDraw()
  // is in milliamps and getActualVelocity() in rpm.
  const std::optional<QCurrent> draw = conveyor.current();
  const std::optional<QAngularVelocity> speed = conveyor.velocity();
  return draw && speed && *draw > 2.2_A &&
         mclib::units::abs(*speed) < 60_rpm;
}

void runConveyor() {
  // Temperature has no typed getter. It is degrees C, and NaN when the motor
  // is not answering. NaN fails every comparison, so a dead motor never
  // counts as hot.
  const QVoltage full = conveyor.getTemperature() >= 50.0 ? 6_V : 12_V;

  // setVoltage() takes a typed voltage or plain volts, and clamps to 12 V
  // either way. setPercent() takes a fraction from -1 to 1, not a percentage.
  if (master.getDigital(DigitalButton::R1)) {
    conveyor.setVoltage(full);
  } else if (master.getDigital(DigitalButton::R2)) {
    conveyor.setVoltage(-full);
  } else {
    conveyor.stop();
  }

  const std::optional<QAngle> turned = checked("CONVEYOR", conveyor.position());
  if (report_due && turned) {
    std::printf("conveyor turned %.0f deg, efficiency %.0f%%\n", turned->deg(),
                conveyor_pros.get_efficiency());
  }
}

// ---------------------------------------------------------------------------
// MotorGroup, Rotation and LimitSwitch: the lift.
// ---------------------------------------------------------------------------
const QAngle lift_top = 115_deg;

void setUpLift() {
  lift_motors.setBrakeMode(mclib::device::BrakeMode::Hold);
  // The lift has to be resting on its bottom stop when the program starts.
  lift_motors.tarePosition();
  lift_sensor.resetPosition();
  if (!lift_sensor.isInstalled()) {
    std::printf("no rotation sensor on the lift pivot's port\n");
  }
}

void checkLiftMotors() {
  // averagePosition(), averageVelocity() and averageCurrent() are empty when
  // any motor of the group is not answering. The plain getAverage...()
  // getters skip the dead motor and average the rest, which hides it.
  const bool all_answering =
      checked("LIFT", lift_motors.averageCurrent()).has_value();
  if (!report_due) {
    return;
  }

  // positions(), velocities() and currents() have one entry per motor, in the
  // order the ports were given. The empty one is the dead motor. A lift on
  // one motor of two still moves, so the driver keeps control and is told
  // which motor it is.
  if (!all_answering) {
    const std::vector<std::optional<QCurrent>> draws = lift_motors.currents();
    for (std::size_t motor = 0; motor < draws.size(); ++motor) {
      if (!draws[motor]) {
        std::printf("lift motor %d of %d is not answering\n",
                    static_cast<int>(motor) + 1,
                    static_cast<int>(draws.size()));
      }
    }
  }

  // The per-motor plain getters keep PROS's own error value, which is
  // infinity for a temperature and passes a `>=` test. Check isfinite first.
  for (const double celsius : lift_motors.getTemperatures()) {
    if (std::isfinite(celsius) && celsius >= 50.0) {
      std::printf("a lift motor is at %.0f C\n", celsius);
    }
  }
}

void runLift() {
  // analog() is the stick as a fraction from -1 to 1, the range setPercent()
  // takes. getAnalog() is the raw count, -127 to 127.
  double command = master.analog(AnalogAxis::RightY);
  if (std::abs(command) < 0.08) {
    command = 0.0;
  }

  // newPress() is true once per press, isPressed() for as long as the switch
  // is held down.
  if (lift_bottom.newPress()) {
    lift_sensor.resetPosition();
  }
  if (lift_bottom.isPressed()) {
    command = std::max(command, 0.0);
  }

  const std::optional<QAngle> angle =
      checked("LIFT ROT", lift_sensor.position());
  if (!angle) {
    // Without the angle there is no top limit. The driver keeps the lift at
    // half power, and the fault is on the controller screen.
    command *= 0.5;
  } else if (*angle >= lift_top) {
    command = std::min(command, 0.0);
  }

  if (command == 0.0) {
    lift_motors.stop();
  } else {
    lift_motors.setPercent(command);
  }
  checkLiftMotors();

  if (report_due && angle) {
    std::printf("lift at %.1f deg, stick %d of 127\n", angle->deg(),
                static_cast<int>(master.getAnalog(AnalogAxis::RightY)));
  }
}

// ---------------------------------------------------------------------------
// Pneumatic, PneumaticGroup and MockPneumatic. IPneumatic is the interface
// all three share.
// ---------------------------------------------------------------------------

// get_value() is the last state the port accepted, not a sensor reading.
// set_value() only remembers the new state once the write went through, so an
// unchanged get_value() means the write failed: a wrong port, or an unplugged
// expander. For a group, get_value() is its first piston.
void setPneumatic(mclib::device::IPneumatic& pneumatic, const char* name,
                  bool extended) {
  pneumatic.set_value(extended);
  faults.check(name, pneumatic.get_value() == extended);
}

void runPneumatics() {
  // getNewPress() and getNewRelease() are true once per press or release.
  // PROS keeps one flag per button per task, so read each button's presses in
  // one place only.
  if (master.getNewPress(DigitalButton::A)) {
    setPneumatic(claw, "CLAW", !claw.get_value());
  }
  if (master.getNewPress(DigitalButton::Up)) {
    setPneumatic(wings, "WINGS", true);
  }
  if (master.getNewRelease(DigitalButton::Up)) {
    setPneumatic(wings, "WINGS", false);
  }

  if (report_due) {
    const std::vector<bool> wing_is_out = wings.get_all_values();
    std::printf("claw %s, left wing %s, right wing %s\n",
                claw.get_value() ? "shut" : "open",
                wing_is_out[0] ? "out" : "in", wing_is_out[1] ? "out" : "in");
  }
}

// ---------------------------------------------------------------------------
// Optical: sorting pieces by colour.
// ---------------------------------------------------------------------------
void setUpPieceSensor() {
  // While gestures are on, the hue, saturation and brightness reads stop.
  // competition_initialize() turns them on, so this runs when a match starts.
  reportIfRefused("optical gestures off", piece_sensor.disableGesture());
  // The white LED, 0 to 100. Without it the hue moves with the room lighting.
  reportIfRefused("optical LED", piece_sensor.setLedPwm(100));
  // PROS's default of about 100 ms is too slow for a piece passing through a
  // conveyor.
  reportIfRefused("optical integration time",
                  piece_sensor.setIntegrationTimeMs(20));
}

void sortPieces() {
  // isColor() is false when the sensor is not answering, so a dead sensor
  // never claims to see a piece. Then the sorter stays in, every piece
  // passes, and the fault is on the controller screen.
  const std::optional<std::int32_t> nearness =
      checked("OPTICAL", piece_sensor.proximity());

  // Blue is hue 200 to 250. Red wraps past 360: isColor(340, 20, 100). The
  // third number is how near the piece has to be, 0 to 255, bigger is nearer.
  // It keeps the floor's hue from counting when the conveyor is empty.
  setPneumatic(sorter, "SORTER", piece_sensor.isColor(200, 250, 100));

  if (!report_due || !nearness) {
    return;
  }
  const std::optional<double> hue = piece_sensor.hue();
  const std::optional<double> saturation = piece_sensor.saturation();
  const std::optional<double> brightness = piece_sensor.brightness();
  const std::optional<mclib::device::Optical::Rgb> colour = piece_sensor.rgb();
  if (hue && saturation && brightness && colour) {
    std::printf("piece nearness %d, hue %.0f, saturation %.2f, brightness "
                "%.2f, rgb %.0f %.0f %.0f\n",
                static_cast<int>(*nearness), *hue, *saturation, *brightness,
                colour->red, colour->green, colour->blue);
  }
}

// ---------------------------------------------------------------------------
// Inertial and MockIMU.
// ---------------------------------------------------------------------------
void setUpHeading() {
  // Calibrate with the robot still. reset(true) waits about 2 s for it to
  // finish. It returns nothing, so a missing IMU shows up as empty readings.
  imu->reset(true);
  reportIfRefused("second IMU calibration", second_imu.reset(true));
  if (imu->hasConfigurationError()) {
    std::printf("the IMU gain has to be a number above 0\n");
  }
}

void readHeading() {
  if (master.getNewPress(DigitalButton::Y)) {
    imu->setRotation(0_deg);
    reportIfRefused("second IMU zero", second_imu.set_rotation(0.0));
  }

  // rotation() keeps counting past 360. heading() wraps into 0 to 360. Both
  // are empty while the IMU calibrates, when it is unplugged, and when its
  // gain is wrong.
  const std::optional<QAngle> rotation = checked("IMU", imu->rotation());
  const std::optional<QAngle> heading = imu->heading();

  // MockIMU has the PROS getters, which return a double. NaN is a failed
  // read.
  const double second_deg = second_imu.get_rotation();
  const bool second_answered = faults.check("IMU 2", !std::isnan(second_deg));

  if (rotation && second_answered) {
    // Two IMUs that disagree: one has drifted or been knocked.
    const QAngle apart =
        mclib::units::abs(*rotation - second_deg * mclib::units::degree);
    faults.check("IMU DIFF", apart < 5_deg);
  }
  if (report_due && rotation && heading) {
    std::printf("rotation %.1f deg, heading %.1f deg\n", rotation->deg(),
                heading->deg());
  }
}

// ---------------------------------------------------------------------------
// Distance and AdiUltrasonic.
// ---------------------------------------------------------------------------
void readDistances() {
  // A working distance sensor with nothing in range reports 9999 mm. That is
  // a real answer, not a failure.
  const std::optional<QLength> ahead =
      checked("DISTANCE", front_distance.distance());

  // getConfidence() runs 0 to 63. From a dead port it hands PROS_ERR straight
  // back, which passes this test, so read it only once distance() has
  // answered.
  const bool ahead_is_trusted = ahead && front_distance.getConfidence() >= 32;

  // The ultrasonic is the other way round. Its distance() is empty when no
  // echo came back, which is what open floor gives. Only PROS_ERR from the
  // plain getter means the port is wrong.
  faults.check("SONAR", rear_sonar.getDistanceCm() != PROS_ERR);
  const std::optional<QLength> behind = rear_sonar.distance();

  if (!report_due) {
    return;
  }
  if (ahead_is_trusted) {
    std::printf("%.1f in to whatever is ahead\n", ahead->in());
  }
  if (behind) {
    std::printf("%.0f cm to whatever is behind\n", behind->cm());
  }
}

// ---------------------------------------------------------------------------
// Gps.
// ---------------------------------------------------------------------------
// Inches from the middle of the field, and heading in radians: 0 faces +Y and
// positive is clockwise.
const mclib::Pose2D left_start(-48.0, -60.0, 0.0);
const mclib::Pose2D right_start(48.0, -60.0, 0.0);
bool starts_on_left = true;

void setUpGps() {
  // This sensor faces backward, so it reads 180 degrees away from the robot's
  // heading. The wrapper adds this to every heading it reads.
  gps.setHeadingOffsetDeg(180.0);
  reportIfRefused("GPS data rate", gps.setDataRateMs(10));
}

void readGps() {
  // The pose is all or nothing: empty if x, y or heading failed.
  const std::optional<mclib::Pose2D> pose = checked("GPS", gps.pose());

  // error() is how far out the sensor thinks its position may be. It grows
  // when the sensor cannot see enough of the code strip on the field wall. A
  // fix worse than 3 in is not used.
  const std::optional<QLength> uncertainty = gps.error();
  if (!pose || !uncertainty || *uncertainty > 3_in || !report_due) {
    return;
  }
  std::printf("GPS x %.1f in, y %.1f in, heading %.0f deg, within %.1f in\n",
              pose->x, pose->y, radToDeg(pose->theta), uncertainty->in());
}

// ---------------------------------------------------------------------------
// Vision.
// ---------------------------------------------------------------------------
void setUpGoalCamera() {
  // Signature 1. The seven numbers and the range are what the Vision Utility
  // prints for a signature, in the same order.
  reportIfRefused("vision signature",
                  goal_camera.setSignature(1, 8973, 11143, 10058, -2119, -1053,
                                           -1586, 5.4f));
  // Fixed exposure and white balance. Left on automatic, the same colour
  // reads differently as the robot drives under different lights.
  reportIfRefused("vision exposure", goal_camera.setExposure(60));
  reportIfRefused("vision white balance",
                  goal_camera.setWhiteBalance(0xFFFFFF));
  reportIfRefused("vision LED", goal_camera.setLed(0x00FF00));
}

void readGoalCamera() {
  // largest() is empty both when nothing matches and when the camera is not
  // answering. getExposure() tells the two apart.
  if (!faults.check("VISION", goal_camera.getExposure() != PROS_ERR) ||
      !report_due) {
    return;
  }

  // x and y are pixels from the middle of the picture: +x right, +y down.
  // Dividing by half the width gives -1 at the left edge and +1 at the right.
  if (const std::optional<mclib::device::VisionObject> goal =
          goal_camera.largest(1)) {
    const double half_width = mclib::device::Vision::kFrameWidth / 2.0;
    std::printf("goal %.2f of the way to the right edge, area %d px\n",
                goal->x / half_width, static_cast<int>(goal->area()));
  }
  std::printf("%d objects in view, %d of them signature 1\n",
              static_cast<int>(goal_camera.count()),
              static_cast<int>(goal_camera.objects(1).size()));
}

// ---------------------------------------------------------------------------
// AiVision.
// ---------------------------------------------------------------------------
void setUpPieceCamera() {
  // Pick the detectors to run. AiVisionMode values combine with |.
  reportIfRefused("AI vision detectors",
                  piece_camera.enable(AiVisionMode::All |
                                      AiVisionMode::ColorMerge));
  // Colour 1 is this red (220, 40, 40), give or take 10 degrees of hue and
  // 0.2 of saturation.
  reportIfRefused("AI vision colour",
                  piece_camera.setColor(1, 220, 40, 40, 10.0f, 0.2f));
  reportIfRefused("AI vision tag family",
                  piece_camera.setTagFamily(pros::AivisionTagFamily::tag_21H7));
  reportIfRefused("AI vision white balance",
                  piece_camera.startAutoWhiteBalance());
}

void readPieceCamera() {
  // An empty result means nothing was seen, or the camera is not answering.
  // enabledMask() tells the two apart.
  if (!faults.check("AIVISION", piece_camera.enabledMask() != PROS_ERR) ||
      !report_due) {
    return;
  }

  // Pixels from the top-left corner of a 320 x 240 picture, so 160 is the
  // middle.
  if (const std::optional<AiVisionObject> piece =
          piece_camera.largest(AiVisionType::Color, 1)) {
    std::printf("nearest red piece is %d px right of centre\n",
                static_cast<int>(piece->center_x) - 160);
  }

  for (const AiVisionObject& seen : piece_camera.objects()) {
    switch (seen.type) {
      case AiVisionType::Color:
      case AiVisionType::Code:
        std::printf("colour or code %d, tilted %.0f deg\n",
                    static_cast<int>(seen.id), seen.angle_deg);
        break;
      case AiVisionType::Object: {
        const std::optional<std::string> kind = piece_camera.className(seen.id);
        std::printf("%s, %d%% sure\n", kind ? kind->c_str() : "unnamed object",
                    static_cast<int>(seen.score));
        break;
      }
      case AiVisionType::Tag:
        std::printf("tag %d, %d px wide\n", static_cast<int>(seen.id),
                    static_cast<int>(seen.width));
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// 3-wire sensors: AdiEncoder, AdiPotentiometer, AdiAnalogIn, Line and Bumper.
// A 3-wire port cannot tell that nothing is plugged into it. These checks
// catch a wrong port letter or a missing expander, not a loose cable: an
// unplugged switch reads as not pressed.
// ---------------------------------------------------------------------------
void setUpThreeWireSensors() {
  reportIfRefused("tracker reset", side_tracker.reset());
  // calibrate() averages the input for about half a second and makes that the
  // zero for getCalibrated(). Keep the robot still while it runs.
  reportIfRefused("light sensor calibration", light_sensor.calibrate());
  // Readings taken on this field: 300 over the white tape, 2600 over the grey
  // tile. The line's threshold becomes the midpoint.
  floor_line.calibrate_midpoint(300, 2600);
}

void readThreeWireSensors() {
  // One encoder tick is one degree, and the count does not wrap.
  const std::optional<QAngle> tracker_angle =
      checked("TRACKER", side_tracker.position());
  // A potentiometer reads an absolute angle, so it survives a power cycle.
  const std::optional<QAngle> claw_opening =
      checked("CLAW POT", claw_pot.angle());
  // raw() is 0 to 4095. fraction() is the same reading as 0 to 1.
  const std::optional<std::int32_t> light =
      checked("LIGHT", light_sensor.raw());
  // get() is false on a failed read, so the failure shows in get_raw().
  const bool line_answered =
      faults.check("LINE", floor_line.get_raw() != PROS_ERR);

  faults.check("BUMPER", back_bumper.getValue() != PROS_ERR);
  if (back_bumper.newPress()) {
    master.rumble(".");
  }

  if (!report_due) {
    return;
  }
  if (tracker_angle) {
    std::printf("tracking wheel rolled %.2f in\n",
                side_wheel.encoderToDistance(*tracker_angle).in());
  }
  if (claw_opening) {
    std::printf("claw at %.0f deg\n", claw_opening->deg());
  }
  if (light) {
    std::printf("light %.2f of full scale, %d from its calibrated zero\n",
                light_sensor.fraction(),
                static_cast<int>(light_sensor.getCalibrated()));
  }
  if (line_answered) {
    std::printf("%s the tape: raw %d, threshold %d\n",
                floor_line.get() ? "over" : "off",
                static_cast<int>(floor_line.get_raw()),
                static_cast<int>(floor_line.get_threshold()));
  }
}

// ---------------------------------------------------------------------------
// AdiLed: a status strip.
// ---------------------------------------------------------------------------
void showStatusStrip() {
  static std::uint32_t shown_colour = 0;
  static bool shown_claw_shut = false;

  // Colours are 0xRRGGBB.
  const std::uint32_t red = 0xFF0000;
  const std::uint32_t amber = 0xFFA000;
  const std::uint32_t green = 0x00FF00;
  const std::uint32_t blue = 0x0000FF;

  const std::uint32_t colour =
      faults.any() ? red : conveyorIsJammed() ? amber : green;
  const bool claw_shut = claw.get_value();

  // Every call writes the whole strip and holds the 3-wire ports while it
  // does, so write on a change, not on every pass.
  if (colour == shown_colour && claw_shut == shown_claw_shut) {
    return;
  }
  bool written = status_strip.setAll(colour) != PROS_ERR;
  if (claw_shut) {
    // setPixel takes the colour first and the position second. Both are
    // plain numbers, so swapping them still compiles.
    const std::uint32_t last_pixel = status_strip.length() - 1;
    written = written && status_strip.setPixel(blue, last_pixel) != PROS_ERR;
  }
  if (faults.check("LED", written)) {
    shown_colour = colour;
    shown_claw_shut = claw_shut;
  }
}

void initialize() {
  showBanner();
  setUpHeading();
  setUpConveyor();
  setUpLift();
  setUpGps();
  setUpGoalCamera();
  setUpPieceCamera();
  setUpThreeWireSensors();
}

void disabled() {
  reportIfRefused("status strip off", status_strip.clear());
}

void competition_initialize() {
  // Before a match, swipe left or right over the optical sensor to say which
  // side the robot starts on.
  reportIfRefused("optical gestures on", piece_sensor.enableGesture());

  using Gesture = mclib::device::Optical::Gesture;
  Gesture previous = Gesture::None;
  while (true) {
    // A swipe stays readable for about half a second, so act once, when it
    // first shows. A sensor that is not answering reads None.
    const Gesture swipe = piece_sensor.gesture();
    if (swipe != previous &&
        (swipe == Gesture::Left || swipe == Gesture::Right)) {
      starts_on_left = swipe == Gesture::Left;
      showOnController(1, starts_on_left ? "START LEFT" : "START RIGHT");
    }
    previous = swipe;
    pros::delay(20);
  }
}

void autonomous() {
  setUpPieceSensor();
  // The GPS finds itself from the field wall alone. Telling it where the
  // robot starts makes its first readings usable sooner.
  reportIfRefused("GPS start pose",
                  gps.setPose(starts_on_left ? left_start : right_start));
}

void opcontrol() {
  setUpPieceSensor();

  std::uint32_t last_report_ms = mclib::time::millis();
  while (true) {
    report_due = mclib::time::hasElapsed(last_report_ms, 500_ms);
    if (report_due) {
      last_report_ms = mclib::time::millis();
    }

    checkController(master);
    checkController(partner);
    runConveyor();
    runLift();
    runPneumatics();
    sortPieces();
    readHeading();
    readDistances();
    readGps();
    readGoalCamera();
    readPieceCamera();
    readThreeWireSensors();
    showStatusStrip();
    faults.show(master);

    pros::delay(10);
  }
}
