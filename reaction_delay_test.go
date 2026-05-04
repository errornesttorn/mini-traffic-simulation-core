package sim

import "testing"

func TestDriverReactionDelayKeepsAcceleratingBeforeBrakeOrHold(t *testing.T) {
	for _, requested := range []carReactionMode{carReactionBrake, carReactionHold} {
		car := Car{
			ID:                 12,
			CurrentSplineID:    3,
			DistanceOnSpline:   17,
			driverReactionMode: carReactionAccelerate,
		}

		mode := applyDriverReactionDelay(&car, requested, 0)
		if mode != carReactionAccelerate {
			t.Fatalf("expected initial transition to keep accelerating, got mode %d", mode)
		}
		if car.driverReactionTimer < driverReactionDelayMinS || car.driverReactionTimer > driverReactionDelayMaxS {
			t.Fatalf("reaction delay %.3f outside configured range %.3f..%.3f", car.driverReactionTimer, driverReactionDelayMinS, driverReactionDelayMaxS)
		}

		delay := car.driverReactionTimer
		mode = applyDriverReactionDelay(&car, requested, delay+0.01)
		if mode != requested {
			t.Fatalf("expected mode %d after delay, got %d", requested, mode)
		}
	}
}

func TestDriverReactionDelayReleasesToHoldBeforeAccelerating(t *testing.T) {
	car := Car{
		ID:                 22,
		CurrentSplineID:    5,
		DistanceOnSpline:   9,
		driverReactionMode: carReactionBrake,
	}

	mode := applyDriverReactionDelay(&car, carReactionAccelerate, 0)
	if mode != carReactionHold {
		t.Fatalf("expected brake release to hold during reaction delay, got mode %d", mode)
	}
	shouldBrake, shouldHold := driverReactionModeDecisions(mode)
	if shouldBrake || !shouldHold {
		t.Fatalf("expected effective hold during release, brake=%v hold=%v", shouldBrake, shouldHold)
	}

	delay := car.driverReactionTimer
	mode = applyDriverReactionDelay(&car, carReactionAccelerate, delay+0.01)
	if mode != carReactionAccelerate {
		t.Fatalf("expected acceleration after delay, got mode %d", mode)
	}
}

func TestDriverReactionDelayDoesNotDelayBrakeHoldSwitches(t *testing.T) {
	car := Car{driverReactionMode: carReactionBrake}
	if mode := applyDriverReactionDelay(&car, carReactionHold, 0); mode != carReactionHold {
		t.Fatalf("expected immediate brake-to-hold switch, got mode %d", mode)
	}
	if car.driverReactionTimer != 0 {
		t.Fatalf("expected brake-to-hold switch to clear timer, got %.3f", car.driverReactionTimer)
	}

	if mode := applyDriverReactionDelay(&car, carReactionBrake, 0); mode != carReactionBrake {
		t.Fatalf("expected immediate hold-to-brake switch, got mode %d", mode)
	}
	if car.driverReactionTimer != 0 {
		t.Fatalf("expected hold-to-brake switch to clear timer, got %.3f", car.driverReactionTimer)
	}
}

func TestDriverReactionDelayCanReturnToBrakeDuringAccelerationRelease(t *testing.T) {
	car := Car{
		ID:                 31,
		CurrentSplineID:    8,
		driverReactionMode: carReactionBrake,
	}

	if mode := applyDriverReactionDelay(&car, carReactionAccelerate, 0); mode != carReactionHold {
		t.Fatalf("expected release to hold during reaction delay, got mode %d", mode)
	}
	if mode := applyDriverReactionDelay(&car, carReactionBrake, 0); mode != carReactionBrake {
		t.Fatalf("expected immediate return to brake, got mode %d", mode)
	}
	if car.driverReactionTimer != 0 {
		t.Fatalf("expected immediate return to brake to clear timer, got %.3f", car.driverReactionTimer)
	}
}
