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
