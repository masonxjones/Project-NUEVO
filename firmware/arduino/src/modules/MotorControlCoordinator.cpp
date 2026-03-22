#include "MotorControlCoordinator.h"

#include <util/atomic.h>

#include "../config.h"
#include "../drivers/DCMotor.h"

volatile uint32_t MotorControlCoordinator::roundCount_ = 0;
volatile uint32_t MotorControlCoordinator::requestedRound_ = 0;
volatile uint32_t MotorControlCoordinator::computedRound_ = 0;
volatile uint32_t MotorControlCoordinator::appliedRound_ = 0;
volatile uint8_t MotorControlCoordinator::currentSlot_ = 0;
volatile uint8_t MotorControlCoordinator::isrSlot_ = 0;
volatile uint32_t MotorControlCoordinator::roundStartUs_ = 0;
volatile uint8_t MotorControlCoordinator::computeSeq_ = 0;
volatile uint8_t MotorControlCoordinator::appliedSeq_ = 0;
volatile uint32_t MotorControlCoordinator::missedRoundCount_ = 0;
volatile uint32_t MotorControlCoordinator::lateComputeCount_ = 0;
volatile uint32_t MotorControlCoordinator::reusedOutputCount_ = 0;
volatile uint32_t MotorControlCoordinator::crossRoundComputeCount_ = 0;
volatile uint32_t MotorControlCoordinator::requestedSeqBySlot_[MotorControlCoordinator::kMaxMotorSlots] = {};
volatile uint32_t MotorControlCoordinator::preparedSeqBySlot_[MotorControlCoordinator::kMaxMotorSlots] = {};
volatile uint8_t MotorControlCoordinator::pendingMask_ = 0;
volatile bool MotorControlCoordinator::computeBusy_ = false;
volatile uint8_t MotorControlCoordinator::computeSlot_ = 0;
volatile uint32_t MotorControlCoordinator::computeRequestSeq_ = 0;

void MotorControlCoordinator::clearPendingState() {
    pendingMask_ = 0;
    computeBusy_ = false;
    computeSlot_ = 0;
    computeRequestSeq_ = 0;
    isrSlot_ = 0;
    currentSlot_ = 0;
    for (uint8_t i = 0; i < kMaxMotorSlots; i++) {
        requestedSeqBySlot_[i] = 0;
        preparedSeqBySlot_[i] = 0;
    }
}

void MotorControlCoordinator::init() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        roundCount_ = 0;
        requestedRound_ = 0;
        computedRound_ = 0;
        appliedRound_ = 0;
        currentSlot_ = 0;
        isrSlot_ = 0;
        roundStartUs_ = 0;
        computeSeq_ = 0;
        appliedSeq_ = 0;
        missedRoundCount_ = 0;
        lateComputeCount_ = 0;
        reusedOutputCount_ = 0;
        crossRoundComputeCount_ = 0;
        clearPendingState();
    }
}

void MotorControlCoordinator::snapshot(uint32_t &roundCount,
                                       uint32_t &requestedRound,
                                       uint32_t &computedRound,
                                       uint32_t &appliedRound,
                                       uint8_t &slot,
                                       uint8_t &computeSeq,
                                       uint8_t &appliedSeq,
                                       uint32_t &missedRoundCount,
                                       uint32_t &lateComputeCount,
                                       uint32_t &reusedOutputCount,
                                       uint32_t &crossRoundComputeCount,
                                       bool &computeBusy) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        roundCount = roundCount_;
        requestedRound = requestedRound_;
        computedRound = computedRound_;
        appliedRound = appliedRound_;
        slot = currentSlot_;
        computeSeq = computeSeq_;
        appliedSeq = appliedSeq_;
        missedRoundCount = missedRoundCount_;
        lateComputeCount = lateComputeCount_;
        reusedOutputCount = reusedOutputCount_;
        crossRoundComputeCount = crossRoundComputeCount_;
        computeBusy = computeBusy_;
    }
}

uint8_t MotorControlCoordinator::getComputeSeq() {
    uint8_t value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = computeSeq_;
    }
    return value;
}

bool MotorControlCoordinator::isComputeBusy() {
    bool value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = computeBusy_;
    }
    return value;
}

bool MotorControlCoordinator::hasPendingRound() {
    bool value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = (pendingMask_ != 0U) || computeBusy_;
    }
    return value;
}

uint16_t MotorControlCoordinator::servicePidIsrSlice(DCMotor *motors,
                                                     uint8_t motorCount,
                                                     bool running) {
    if (motors == nullptr || motorCount == 0U) {
        return 0U;
    }

    const uint8_t slot = isrSlot_;
    currentSlot_ = slot;

    if (slot == 0U) {
        roundStartUs_ = micros();
        if (running) {
            roundCount_++;
        }
    }

    if (!running) {
        clearPendingState();
        requestedRound_ = roundCount_;
        computedRound_ = roundCount_;
        appliedRound_ = roundCount_;
        appliedSeq_ = computeSeq_;
    } else {
        const uint8_t slotBit = (uint8_t)(1U << slot);
        const uint32_t expectedSeq = requestedSeqBySlot_[slot];

        if (expectedSeq != 0U) {
            if (preparedSeqBySlot_[slot] == expectedSeq) {
                motors[slot].publishStagedOutputISR();
                appliedRound_ = expectedSeq;
                appliedSeq_ = (uint8_t)appliedRound_;
            } else {
                reusedOutputCount_++;
            }
        }

        motors[slot].latchFeedbackISR();
        motors[slot].update();

        if (motors[slot].isEnabled()) {
            if ((pendingMask_ & slotBit) != 0U || (computeBusy_ && computeSlot_ == slot)) {
                missedRoundCount_++;
            }
            requestedRound_++;
            requestedSeqBySlot_[slot] = requestedRound_;
            pendingMask_ |= slotBit;
        } else {
            pendingMask_ &= (uint8_t)~slotBit;
            requestedSeqBySlot_[slot] = 0U;
            preparedSeqBySlot_[slot] = 0U;
        }
    }

    uint16_t completedRoundUs = 0U;
    if (slot == (uint8_t)(motorCount - 1U) && running) {
        uint32_t elapsedUs = micros() - roundStartUs_;
        completedRoundUs = (elapsedUs > 0xFFFFUL) ? 0xFFFFU : (uint16_t)elapsedUs;
    }

    isrSlot_ = (uint8_t)((slot + 1U) % motorCount);
    currentSlot_ = isrSlot_;
    return completedRoundUs;
}

void MotorControlCoordinator::resetForNonRunningTask() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        clearPendingState();
        requestedRound_ = roundCount_;
        computedRound_ = roundCount_;
        appliedRound_ = roundCount_;
    }
}

bool MotorControlCoordinator::beginCompute(uint32_t &requestedRound, uint8_t &slotSnapshot) {
    bool shouldRun = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (pendingMask_ != 0U && !computeBusy_) {
            uint8_t oldestSlot = 0xFFU;
            uint32_t oldestSeq = 0xFFFFFFFFUL;
            for (uint8_t i = 0; i < kMaxMotorSlots; i++) {
                const uint8_t slotBit = (uint8_t)(1U << i);
                if ((pendingMask_ & slotBit) == 0U) {
                    continue;
                }
                const uint32_t seq = requestedSeqBySlot_[i];
                if (seq != 0U && seq < oldestSeq) {
                    oldestSeq = seq;
                    oldestSlot = i;
                }
            }

            if (oldestSlot != 0xFFU) {
                requestedRound = oldestSeq;
                slotSnapshot = oldestSlot;
                computeSlot_ = oldestSlot;
                computeRequestSeq_ = oldestSeq;
                currentSlot_ = oldestSlot;
                shouldRun = true;
            }
        }

        if (shouldRun) {
            computeBusy_ = true;
        }
    }

    if (shouldRun && (requestedRound + (uint32_t)NUM_DC_MOTORS) <= requestedRound_) {
        lateComputeCount_++;
    }

    return shouldRun;
}

void MotorControlCoordinator::finishCompute(uint32_t requestedRound) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        const uint8_t slot = computeSlot_;
        const uint8_t slotBit = (uint8_t)(1U << slot);

        if (computeRequestSeq_ != requestedRound) {
            computeBusy_ = false;
            computeSlot_ = 0U;
            computeRequestSeq_ = 0U;
            return;
        }

        if (requestedSeqBySlot_[slot] != requestedRound) {
            crossRoundComputeCount_++;
        } else {
            preparedSeqBySlot_[slot] = requestedRound;
            pendingMask_ &= (uint8_t)~slotBit;
            computedRound_ = requestedRound;
            computeSeq_ = (uint8_t)computedRound_;
        }

        computeBusy_ = false;
        computeSlot_ = 0U;
        computeRequestSeq_ = 0U;
    }
}
