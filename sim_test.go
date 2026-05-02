package sim

import "testing"

func TestSettleCarKeepsStableHeadingAtNearZeroSpeed(t *testing.T) {
	spline := straightSpline(1, 0, 0, 40, 0)
	frontPos, tangent := sampleSplineAtDistance(&spline, 10)
	car := Car{
		ID:                  1,
		RouteID:             1,
		CurrentSplineID:     spline.ID,
		DestinationSplineID: spline.ID,
		DistanceOnSpline:    10,
		RearPosition:        vecSub(frontPos, vecScale(tangent, 4.6*defaultWheelbaseFrac)),
		PrevFrontPosition:   vecSub(frontPos, NewVec2(0, 0.25)),
		Heading:             tangent,
		Speed:               frontPivotHeadingMinSpeedMPS * 0.5,
		MaxSpeed:            12,
		Accel:               4,
		Length:              4.6,
		Width:               1.9,
		FrontPivotFrac:      defaultFrontPivotFrac,
		RearPivotFrac:       defaultRearPivotFrac,
	}

	var cursor splineSampleCursor
	cursor.splineID = -1
	got := settleCarOnCurrentSpline(car, &spline, 1.0/60.0, &cursor)

	if dot(got.Heading, tangent) < 0.999 {
		t.Fatalf("expected near-zero speed to keep stable heading, got (%.4f, %.4f)", got.Heading.X, got.Heading.Y)
	}
}

func TestSettleCarUpdatesHeadingAfterMeaningfulFrontPivotMove(t *testing.T) {
	spline := straightSpline(1, 0, 0, 40, 0)
	frontPos, tangent := sampleSplineAtDistance(&spline, 10)
	expected := NewVec2(0, 1)
	car := Car{
		ID:                  1,
		RouteID:             1,
		CurrentSplineID:     spline.ID,
		DestinationSplineID: spline.ID,
		DistanceOnSpline:    10,
		RearPosition:        vecSub(frontPos, vecScale(tangent, 4.6*defaultWheelbaseFrac)),
		PrevFrontPosition:   vecSub(frontPos, vecScale(expected, frontPivotHeadingMinDisplacement*2)),
		Heading:             tangent,
		Speed:               frontPivotHeadingMinSpeedMPS * 2,
		MaxSpeed:            12,
		Accel:               4,
		Length:              4.6,
		Width:               1.9,
		FrontPivotFrac:      defaultFrontPivotFrac,
		RearPivotFrac:       defaultRearPivotFrac,
	}

	var cursor splineSampleCursor
	cursor.splineID = -1
	got := settleCarOnCurrentSpline(car, &spline, 1.0/60.0, &cursor)

	if dot(got.Heading, expected) < 0.999 {
		t.Fatalf("expected meaningful movement to update heading, got (%.4f, %.4f)", got.Heading.X, got.Heading.Y)
	}
}
