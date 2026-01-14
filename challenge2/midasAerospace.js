let timeSpent = 0
brick.showString("Hello world", 1)
forever(function () {
    timeSpent += 1
    control.waitMicros(4)
    if (timeSpent >= 15) {

    }
})
forever(function () {
    motors.largeAB.tank(60, 60)
    if (sensors.touch1.isPressed()) {
        motors.stopAll()
        pause(100)
        motors.mediumD.run(-60)
        while (true) {
            motors.stopAll()
        }
    }
})
forever(function () {
    if (sensors.color3.isColorDetected(ColorSensorColor.Blue)) {
        motors.stopAll()
        pause(100)
        motors.mediumD.run(-60)
        while (true) {
            motors.stopAll()
        }
    } else if (sensors.color3.isColorDetected(ColorSensorColor.Yellow) || sensors.color3.isColorDetected(ColorSensorColor.Green)) {

    }
})
