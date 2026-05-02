package sim

import "testing"

func TestLaneChangeSpecCooldownsBiasByVehicleSpecs(t *testing.T) {
	slow := Car{MaxSpeed: 18, Accel: 1.5}
	average := Car{MaxSpeed: laneChangeSpecRefSpeedMPS, Accel: laneChangeSpecRefAccel}
	fast := Car{MaxSpeed: 36, Accel: 4.5}

	if got := preferenceLaneChangeCooldownS(slow); got >= preferenceLaneChangeCooldownS(fast) {
		t.Fatalf("slow car preference cooldown %.3f should be lower than fast car %.3f", got, preferenceLaneChangeCooldownS(fast))
	}
	if got := overtakeLaneChangeCooldownS(fast); got >= overtakeLaneChangeCooldownS(slow) {
		t.Fatalf("fast car overtake cooldown %.3f should be lower than slow car %.3f", got, overtakeLaneChangeCooldownS(slow))
	}
	if got := overtakeSlowThresholdForCar(fast); got >= overtakeSlowThresholdForCar(slow) {
		t.Fatalf("fast car overtake threshold %.3f should be lower than slow car %.3f", got, overtakeSlowThresholdForCar(slow))
	}

	minPreference := preferenceChangeCooldownS * laneChangeSpecMinFactor
	maxOvertake := overtakeCooldownS * laneChangeSpecMaxFactor
	if got := preferenceLaneChangeCooldownS(slow); got < minPreference {
		t.Fatalf("slow car preference cooldown %.3f should stay above bounded minimum %.3f", got, minPreference)
	}
	if got := overtakeLaneChangeCooldownS(slow); got > maxOvertake {
		t.Fatalf("slow car overtake cooldown %.3f should stay below bounded maximum %.3f", got, maxOvertake)
	}
	if got := preferenceLaneChangeCooldownS(average); absf(got-preferenceChangeCooldownS) > 1e-4 {
		t.Fatalf("average car preference cooldown %.3f should match base %.3f", got, preferenceChangeCooldownS)
	}
}

func TestForcedLaneChangeApproachSpeedCapDeclinesTowardDeadline(t *testing.T) {
	car := Car{
		DesiredLaneSplineID: 2,
		DesiredLaneDeadline: 100,
		Accel:               3,
	}

	far, ok := forcedLaneChangeApproachSpeedCap(car)
	if !ok {
		t.Fatal("expected cap for desired lane change")
	}
	car.DistanceOnSpline = 80
	near, ok := forcedLaneChangeApproachSpeedCap(car)
	if !ok {
		t.Fatal("expected cap near desired lane deadline")
	}
	car.DistanceOnSpline = 100
	atDeadline, ok := forcedLaneChangeApproachSpeedCap(car)
	if !ok {
		t.Fatal("expected cap at desired lane deadline")
	}

	if !(far > near && near > laneChangeForcedSpeedMPS) {
		t.Fatalf("expected cap to taper down toward forced speed, far %.3f near %.3f forced %.3f", far, near, laneChangeForcedSpeedMPS)
	}
	if absf(atDeadline-laneChangeForcedSpeedMPS) > 1e-4 {
		t.Fatalf("expected cap at deadline %.3f to equal forced speed %.3f", atDeadline, laneChangeForcedSpeedMPS)
	}
}

func TestForcedLaneChangeSpeedUpdateStartsSlowingBeforeOldBrakePoint(t *testing.T) {
	spline := NewSpline(1, NewVec2(0, 0), NewVec2(70, 0), NewVec2(130, 0), NewVec2(200, 0))
	graph := NewRoadGraph([]Spline{spline}, nil)
	car := Car{
		CurrentSplineID:      1,
		DestinationSplineID:  1,
		DesiredLaneSplineID:  2,
		DesiredLaneDeadline:  100,
		DistanceOnSpline:     60,
		Speed:                15,
		MaxSpeed:             15,
		Accel:                3,
		CurveSpeedMultiplier: 1,
		LaneChangeSplineID:   -1,
		AfterSplineID:        -1,
	}

	normalDecel := car.Accel * 1.5 * 0.9
	oldBrakeDist := (car.Speed*car.Speed - laneChangeForcedSpeedMPS*laneChangeForcedSpeedMPS) / (2 * normalDecel)
	if remaining := car.DesiredLaneDeadline - car.DistanceOnSpline; remaining <= oldBrakeDist {
		t.Fatalf("test setup must be before old brake point, remaining %.3f old brake dist %.3f", remaining, oldBrakeDist)
	}

	applyCurrentSplineSpeedUpdate(&car, Route{VehicleKind: VehicleCar}, &spline, graph, nil, nil, 9999, false, 1)
	if car.Speed >= 15 {
		t.Fatalf("expected desired-lane car to start slowing before old brake point, got speed %.3f", car.Speed)
	}
}
