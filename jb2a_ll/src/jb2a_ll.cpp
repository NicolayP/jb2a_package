/*  Layout:
 * struct of the robot: mainly servos related to the same rotation
 *  and the orientation (rotation direction).
 * subscribe to /angle (this name might change)
 * use the structure to convert angles of every joint to
 * angle in every servo's frame.
 * publisher on /absolute-angle
 * publish the angle values to execute them on the real robot.
 */
