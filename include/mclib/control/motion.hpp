// mclib
#pragma once

#include "mclib/units/units.hpp"

#include <limits>

/**
 * @file motion.hpp
 * @brief The blocking motion routines, in units.
 *
 * These are the primitives an autonomous is written out of. Each one blocks on
 * its own task until it settles, times out, or someone calls
 * `mclib::control::requestCancel(CancelToken::Motion)`.
 *
 * ## Frame
 *
 * Positions are field-frame inches and headings are **absolute field
 * headings**, not deltas: `turnToAngle(90_deg, 1_s)` turns to face 90 deg, it
 * does not turn by 90 deg. The frame is compass - 0 deg is +Y, clockwise is
 * positive - and is documented in full in `mclib/math.hpp`.
 *
 * ## Units
 *
 * Every parameter that has a dimension now carries it in its type. The two
 * kinds of parameter that stay bare are:
 *
 * - **Sign flags.** `direction`, `dir` and `drive_direction` are +1 / -1
 *   selectors. They are not quantities, so they are not typed.
 * - **`dlead`.** The boomerang lead factor is a genuine ratio in [0, 1].
 *
 * ## `min_speed` and the -1 sentinel
 *
 * Every routine takes `min_speed`, a floor on the output voltage, defaulting
 * to `-1_V`. Negative means "use the `min_output` global" - the stiction
 * floor, **1.5 V of a 12 V rail**, applied unconditionally in
 * `turnToAngle()`'s chained branches, in `swing()` and in `turnToPoint()`, and
 * gated behind chaining elsewhere. It was 10 V - 83% of the rail as a
 * *minimum* - which is almost certainly a leftover from a 0..127 output scale;
 * see the note on `min_output` in `mclib/config.hpp`.
 */

/**
 * @brief The `reset_heading` value that means "leave the IMU alone".
 *
 * `wallReset()` takes an absolute heading to stamp onto the IMU once the wall
 * has been found. Passing this instead keeps whatever heading the IMU already
 * reports. It is a NaN, so it compares false against everything, including
 * itself - test it with `std::isnan(angle.rad())`, never with `==`.
 */
inline constexpr QAngle keep_current_heading =
    QAngle::fromBase(std::numeric_limits<double>::quiet_NaN());

/**
 * @brief Turn in place to an absolute field heading.
 *
 * @param turn_angle An **absolute** compass heading, not a delta. Normalised
 *                   to within +/-180 deg of the current heading before use.
 * @param time_limit Give up after this long.
 * @param exit       True stops and holds at the end. False runs until the
 *                   heading passes @p turn_angle and leaves the drive
 *                   energised, for chaining into the next motion.
 * @param max_output Voltage cap.
 * @param min_speed  Voltage floor; negative selects the `min_output` default.
 *                   Only the `exit == false` branches apply it.
 */
void turnToAngle(QAngle turn_angle,
                 QTime time_limit,
                 bool exit = true,
                 QVoltage max_output = 12.0 * mclib::units::volt,
                 QVoltage min_speed = -1.0 * mclib::units::volt);

/**
 * @brief Drive straight a relative distance, holding the current heading.
 *
 * Distance is measured off the drive encoders, not the odometry, so it is a
 * displacement along the robot's own path rather than a field position.
 *
 * @param distance   How far to travel. Negative drives backward.
 * @param time_limit Give up after this long.
 * @param exit       True ramps to 0 V and holds at the end. False leaves the
 *                   drive energised for chaining.
 * @param max_output Voltage cap.
 * @param min_speed  Voltage floor; negative selects the `min_output` default.
 *                   Gated off unless the caller asks for it or the motion is
 *                   chained.
 */
void driveTo(QLength distance,
             QTime time_limit,
             bool exit = true,
             QVoltage max_output = 12.0 * mclib::units::volt,
             QVoltage min_speed = -1.0 * mclib::units::volt);

/**
 * @brief Drive a constant-radius arc to an absolute field heading.
 *
 * @param result_angle The **absolute** compass heading to finish on.
 * @param center_radius Radius of the arc, measured to the centre of the robot.
 *                      **Its sign selects the direction**: positive curves
 *                      right, negative curves left. The magnitude must exceed
 *                      half the track width or the inner wheel arc goes
 *                      negative.
 * @param time_limit   Give up after this long.
 * @param exit         True stops and holds at the end.
 * @param max_output   Voltage cap.
 * @param min_speed    Voltage floor; negative selects the `min_output` default.
 * @param reverse      Forces the drive direction backward regardless of which
 *                     way the heading has to move.
 */
void curveCircle(QAngle result_angle,
                 QLength center_radius,
                 QTime time_limit,
                 bool exit = true,
                 QVoltage max_output = 12.0 * mclib::units::volt,
                 QVoltage min_speed = -1.0 * mclib::units::volt,
                 bool reverse = false);

/// @brief `curveCircle()` with @p reverse forced on. See curveCircle().
void curveCircleReverse(QAngle result_angle,
                        QLength center_radius,
                        QTime time_limit,
                        bool exit = true,
                        QVoltage max_output = 12.0 * mclib::units::volt,
                        QVoltage min_speed = -1.0 * mclib::units::volt);

/**
 * @brief Turn about one locked tread to an absolute field heading.
 *
 * @param swing_angle     The **absolute** compass heading to finish on.
 * @param drive_direction +1 swings forward, -1 swings backward. A sign flag,
 *                        not a quantity; it is a `double` because it multiplies
 *                        the output voltage directly.
 * @param time_limit      Give up after this long.
 * @param exit            True stops and holds at the end.
 * @param max_output      Voltage cap.
 * @param min_speed       Voltage floor; negative selects the `min_output`
 *                        default, which this routine applies unconditionally on
 *                        the chained branches.
 */
void swing(QAngle swing_angle,
           double drive_direction,
           QTime time_limit,
           bool exit = true,
           QVoltage max_output = 12.0 * mclib::units::volt,
           QVoltage min_speed = -1.0 * mclib::units::volt);

/**
 * @brief Hold the heading in `RobotState::correctAngleDeg()` forever.
 *
 * Runs on its own task alongside the motions rather than instead of them: it
 * only drives while `RobotState::isTurning()` is false, and it stops when
 * asked through `CancelToken::HeadingCorrection`, which is a separate token
 * from the one the motions use.
 */
void correctHeading();

/**
 * @brief Drive into a wall until the drive stalls, then adopt a known pose.
 *
 * @param reset_x            Field X the wall puts the robot at.
 * @param reset_y            Field Y the wall puts the robot at.
 * @param reset_heading      Absolute heading to stamp onto the IMU, or
 *                           #keep_current_heading to leave the IMU alone.
 * @param drive_power        Voltage to push with. Negative drives backward.
 * @param time_limit         Give up after this long and reset anyway.
 * @param current_threshold  Stall is declared above this average motor current.
 * @param velocity_threshold ...and below this average motor speed, for five
 *                           consecutive 10 ms ticks.
 */
void wallReset(QLength reset_x,
               QLength reset_y,
               QAngle reset_heading,
               QVoltage drive_power,
               QTime time_limit,
               QCurrent current_threshold = 2500.0 * mclib::units::milliampere,
               QAngularVelocity velocity_threshold = 5.0 * mclib::units::rpm);

/**
 * @brief Turn in place to face a field point, re-aiming as the odometry moves.
 *
 * @param x          Field X of the point to face.
 * @param y          Field Y of the point to face.
 * @param direction  +1 faces the point with the front of the robot, -1 with the
 *                   back. A sign flag, not a quantity.
 * @param time_limit Give up after this long.
 * @param min_speed  Voltage floor; negative selects the `min_output` default,
 *                   which this routine applies unconditionally.
 */
void turnToPoint(QLength x,
                 QLength y,
                 int direction = 1,
                 QTime time_limit = 1000.0 * mclib::units::millisecond,
                 QVoltage min_speed = -1.0 * mclib::units::volt);

/**
 * @brief Drive to a field point, steering as it goes.
 *
 * Exits early once the robot crosses the line through the target perpendicular
 * to its own heading, so it does not orbit a point it has already passed.
 *
 * @param x          Field X of the target.
 * @param y          Field Y of the target.
 * @param dir        +1 arrives forward, -1 arrives backward. A sign flag.
 * @param time_limit Give up after this long.
 * @param exit       True ramps to 0 V and holds at the end.
 * @param max_output Voltage cap.
 * @param overturn   True lets heading correction eat into the forward drive
 *                   when the two together exceed @p max_output.
 * @param min_speed  Voltage floor; negative selects the `min_output` default.
 */
void moveToPoint(QLength x,
                 QLength y,
                 int dir,
                 QTime time_limit,
                 bool exit = true,
                 QVoltage max_output = 12.0 * mclib::units::volt,
                 bool overturn = true,
                 QVoltage min_speed = -1.0 * mclib::units::volt);

/**
 * @brief Drive to a field point *and* a final heading, by chasing a carrot.
 *
 * The carrot sits @p dlead of the remaining distance back from the target along
 * @p final_heading, so the path curves into the target rather than arriving at
 * an arbitrary angle.
 *
 * @param x              Field X of the target.
 * @param y              Field Y of the target.
 * @param dir            +1 arrives forward, -1 arrives backward. A sign flag.
 * @param final_heading  The **absolute** compass heading to arrive on.
 * @param dlead          Lead factor, dimensionless, sensibly 0..1. 0 collapses
 *                       the carrot onto the target and makes this
 *                       `moveToPoint()`; larger values swing wider.
 * @param time_limit     Give up after this long.
 * @param exit           True ramps to 0 V and holds at the end.
 * @param max_output     Voltage cap.
 * @param overturn       True lets heading correction eat into the forward drive.
 * @param min_speed      Voltage floor; negative selects the `min_output` default.
 *
 * @warning The slip-speed limiter inside uses `getRadius()`, which is
 *          frame-transposed (`utils.hpp` documents how), and the tuning was
 *          fitted around that. Do not swap it for `mclib::arcRadius()` without
 *          re-tuning.
 */
void boomerang(QLength x,
               QLength y,
               int dir,
               QAngle final_heading,
               double dlead,
               QTime time_limit,
               bool exit = true,
               QVoltage max_output = 12.0 * mclib::units::volt,
               bool overturn = true,
               QVoltage min_speed = -1.0 * mclib::units::volt);
