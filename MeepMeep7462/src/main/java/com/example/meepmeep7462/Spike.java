package com.example.meepmeep7462;

public class Spike {

    // All initial values assume red side, not audience
    final public FieldCoordinate spikePoint  = new FieldCoordinate( 0, 0, 0 );
    final public FieldCoordinate startPose       = new FieldCoordinate( 15, -62.5, 90 );

    public Spike(final Pose _pose, final FieldSide _fieldSize) {

        if(_fieldSize.isBlue) {
            switch (_fieldSize.spike) {
                case 3:
                    spikePoint.x = 3.7;
                    spikePoint.y = -34.5;
                    spikePoint.heading = 115;
                    _pose.aprilTagPose.y = -28.3;
                    break;
                case 1:
                    spikePoint.x = 17.5;
                    spikePoint.y = -33.5;
                    spikePoint.heading = 65;
                    _pose.aprilTagPose.y = -42.4;
                    break;
                default:
                    spikePoint.x = 9.5;
                    spikePoint.y = -33;
                    spikePoint.heading = 105;
                    _pose.aprilTagPose.y = -35.5;
                    break;
            }
        } else {
            switch (_fieldSize.spike) {
                case 1:
                    spikePoint.x = 3.7;
                    spikePoint.y = -34.5;
                    spikePoint.heading = 115;
                    _pose.aprilTagPose.y = -28.3;
                    break;
                case 3:
                    spikePoint.x = 17.5;
                    spikePoint.y = -33.5;
                    spikePoint.heading = 65;
                    _pose.aprilTagPose.y = -42.4;
                    break;
                default:
                    spikePoint.x = 9.5;
                    spikePoint.y = -33;
                    spikePoint.heading = 105;
                    _pose.aprilTagPose.y = -35.5;
                    break;
            }
        }

        if (_fieldSize.isAudience) {
            startPose.x = -39;
            _pose.tempParkPose.y = -10;
            _pose.backdropPose.x = 47.75;

            spikePoint.x -= 46;
            if(_fieldSize.spike == 2){
                spikePoint.x += 4;
                spikePoint.heading = 75;
            }
            _pose.mediaryPose.x = -(_pose.mediaryPose.x + 34);
//            spikePoint.x -= 46;
        }

        if(_fieldSize.isBlue){
            startPose.y *= -1;
            startPose.heading *= -1;

            spikePoint.y *= -1;
            spikePoint.heading *= -1;
            _pose.mediaryPose.y *= -1;
            _pose.mediaryPose.heading *= -1;
            _pose.backdropPose.y *= -1;
            _pose.backdropPose.heading *= -1;
            _pose.centerPose.y *= -1;
            _pose.centerPose.heading *= -1;
            _pose.avoidancePose.y *= -1;
            _pose.avoidancePose.heading *= -1;
            _pose.travelPose.y *= -1;
            _pose.travelPose.heading *= -1;
            _pose.tempParkPose.y *= -1;
            _pose.tempParkPose.heading *= -1;
            _pose.aprilTagPose.y *= -1;
            _pose.aprilTagPose.heading *= -1;
            _pose.pixelPose.y *= -1;
            _pose.pixelPose.heading *= -1;
            _pose.postPixelPose.y *= -1;
            _pose.postPixelPose.heading *= -1;
            _pose.outerTravelPose.y *= -1;
            _pose.outerTravelPose.heading *= -1;
            _pose.outerCenterPose.y *= -1;
            _pose.outerCenterPose.heading *= -1;
            _pose.secondCollectionPose.y *= -1;
            _pose.secondCollectionPose.heading *= -1;
            _pose.finalDepositPose.y *= -1;
            _pose.finalDepositPose.heading *= -1;
            _pose.preSecondCollectionPose.y *= -1;
            _pose.preSecondCollectionPose.heading *= -1;
        }
    }
}
