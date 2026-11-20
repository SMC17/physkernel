/-
  Formal verification of PhysKernel safety state machine invariants:
  1. Safety threshold ordering (0 < warnThreshold < estopThreshold)
  2. Monotonic e_stop latching: once `e_stop` is entered, no subsequent
     e-value observation can transition out of `e_stop`.
-/

namespace PhysKernelProofs

inductive SafetyState where
  | safe
  | warn
  | e_stop
  deriving DecidableEq, Repr

def warnThreshold : Nat := 100000
def estopThreshold : Nat := 1000000000

def stepSafety (s : SafetyState) (eVal : Nat) : SafetyState :=
  match s with
  | .e_stop => .e_stop
  | _ =>
    if eVal ≥ estopThreshold then .e_stop
    else if eVal ≥ warnThreshold then .warn
    else .safe

theorem safety_threshold_ordering : 0 < warnThreshold ∧ warnThreshold < estopThreshold := by
  decide

theorem estop_implies_warn_threshold (eVal : Nat) (h : eVal ≥ estopThreshold) :
    eVal ≥ warnThreshold := by
  have hord : warnThreshold < estopThreshold := safety_threshold_ordering.2
  omega

theorem estop_latching (s : SafetyState) (eVal : Nat) (hs : s = .e_stop) :
    stepSafety s eVal = .e_stop := by
  subst hs
  rfl

theorem estop_latching_multi (es : List Nat) (s : SafetyState) (hs : s = .e_stop) :
    es.foldl stepSafety s = .e_stop := by
  induction es generalizing s with
  | nil => simpa using hs
  | cons e rest ih =>
    simp only [List.foldl_cons]
    exact ih (stepSafety s e) (estop_latching s e hs)

end PhysKernelProofs
