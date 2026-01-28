package org.firstinspires.ftc.teamcode.opmode.autonomous;
// try the custom heading interpolator to move tangentially to and from the goal.
// alternatively just be straight until a point then turn

// this should increase speed.
// leave goal when 3 balls
// sotm last shot (and use a better point)

// sotm on all the time?
// maybe use the zoneutil? when in zone, shoot, start next path right away? maybe too risky
// just remove all shooting time with sotm

// say with new transfer it takes us 0.3s less per shot.
// not sotming 1st shot: 0.3 * 7 = 2.1s
// maybe we save 0.3s on 4 gate intakes = 1.2s
// better tuning+pathing saves us maybe 0.3s per cycle = 1.2s
// it is def possible to get under that time. maybe use turtle pathing as well but honestly i dont like it much
// would rather sotm first shot when conditions right
// for other shots idk just edge the pathing as close as possible, try diff paths to maximize speed
public class BlueClose24 {
}
