# ranger defines a laser sensor in the simulation
# see for details: http://playerstage.sourceforge.net/doc/stage-cvs/group__model__laser.html
define hokuyolaser180 ranger
(
    sensor
    (
        range [ 0.0 2.0 ]
        fov 360.0
        samples 721
    )

    #samples 90 # still useful but much faster to compute

    # generic model properties
    # the following block directives generates the visual layout of the sensor
    # sensor image https://www.hokuyo-aut.jp/search/single.php?serial=169

    size [ 0.07 0.07 0.05 ] # dimensions from LMS200 data sheet

    # gray base
    block
    (
        points 4
        point[0] [0 0]
        point[1] [0 0.03]
        point[2] [0.03 0.03]
        point[3] [0.03 0]
        z [0.00 0.05]
        color "gray"
    )

    # black cylinder
    block
    (

        points 8
        point[0] [0.01 0.005]
        point[1] [0.02 0.005]
        point[2] [0.025 0.01]
        point[3] [0.025 0.02]
        point[4] [0.02 0.025]
        point[5] [0.01 0.025]
        point[6] [0.005 0.02]
        point[7] [0.005 0.01]
        z [0.05 0.12]
        color "black"
    )

    # orange top
    block
    (
        points 4
        point[0] [0.005 0.005]
        point[1] [0.025 0.005]
        point[2] [0.025 0.025]
        point[3] [0.005 0.025]
        z [0.12 0.14]
        color "blue"
    )

    # black top
    block
    (
        points 4
        point[0] [0.005 0.005]
        point[1] [0.025 0.005]
        point[2] [0.025 0.025]
        point[3] [0.005 0.025]
        z [0.14 0.15]
        color "black"
    )
)

# our robot definition
# see for details: http://playerstage.sourceforge.net/doc/stage-cvs/group__model__position.html
define roomba position
(
    # localization "odom"         		# Change to "gps" to have impossibly perfect, global odometry
    # odom_error [0.03 0.03 0 0 0 0.02]

    localization "gps"
    localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0).

    size [0.33 0.33 0.1]
    drive "omni"
    gui_nose 1


    # this block approximates the circular shape of a Roomba
    block(
        points 16
        point[0] [ 0.225 0.000 ]
        point[1] [ 0.208 0.086 ]
        point[2] [ 0.159 0.159 ]
        point[3] [ 0.086 0.208 ]
        point[4] [ 0.000 0.225 ]
        point[5] [ -0.086 0.208 ]
        point[6] [ -0.159 0.159 ]
        point[7] [ -0.208 0.086 ]
        point[8] [ -0.225 0.000 ]
        point[9] [ -0.208 -0.086 ]
        point[10] [ -0.159 -0.159 ]
        point[11] [ -0.086 -0.208 ]
        point[12] [ -0.000 -0.225 ]
        point[13] [ 0.086 -0.208 ]
        point[14] [ 0.159 -0.159 ]
        point[15] [ 0.208 -0.086 ]
        z [0 0.1]
    )

    # the footprint color is the same as the robot base
    #color "gray90"
    color "red"

    hokuyolaser180(pose [0.0 0.0 0.0 0.0])
)
