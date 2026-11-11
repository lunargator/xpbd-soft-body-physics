using SoftBodyPhysics.Core;

namespace SoftBodyPhysics.Constraints {
    public interface IConstraint {
        void ResetLambda();

        void Solve(DeformableBody body, float stepSeconds);
    }
}
