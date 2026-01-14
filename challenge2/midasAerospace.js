let tooLong = 0
let timeSpent = 0
brick.showString("Hello world", 1)
tooLong = 0
forever(function () {
    if (sensors.color3.isColorDetected(ColorSensorColor.Blue)) {
        motors.stopAll()
        pause(100)
        tooLong = 1
        motors.stopAll()
        pause(100)
        while (true) {
            motors.largeAB.run(-60)
        }
    } else if (sensors.color3.isColorDetected(ColorSensorColor.Yellow) || sensors.color3.isColorDetected(ColorSensorColor.Green)) {
    	
    }
})
forever(function () {
    timeSpent += 1
    pause(1000)
    if (timeSpent >= 15) {
        tooLong = 1
    }
})
forever(function () {
    motors.largeAD.tank(60, 60)
    pause(100)
    if (tooLong == 1 || sensors.touch1.isPressed()) {
        motors.mediumD.run(-60)
        while (true) {
            motors.stopAll()
        }
    }
})
