#pragma once

#include <Collision/CollisionMesh.h>
#include <Energy/Collision/VertexFaceSqrtCollision.h>
#include <Energy/Strand/DiscreteElasticRod.h>
#include <Integrator/StrandIntegrator.h>
#include <StrandMesh.h>

class ForwardEulerStrand : public StrandIntegrator {
public:
  Real collisionStiffness = 10;
  Real collisionEpsilon = 0.2;

  std::unique_ptr<VertexFaceSqrtCollision> vertexFaceSqrtCollision;

  ForwardEulerStrand(std::shared_ptr<StrandMesh> strandMesh,
                     std::shared_ptr<DiscreteElasticRod> material,
                     Real dt = 1.0 / 3000.0, Real collisionStiffness = 10,
                     Real collisionEpsilon = 0.2)
      : StrandIntegrator(std::move(strandMesh), std::move(material), dt),
        collisionStiffness(collisionStiffness),
        collisionEpsilon(collisionEpsilon) {
    vertexFaceSqrtCollision = std::make_unique<VertexFaceSqrtCollision>(
        collisionStiffness, collisionEpsilon);
  }

  INLINE void Step(bool bump = false) override {
    Vec<Real> R = this->strandMesh->ComputeMaterialForces();

    std::vector<Vec12<Real>> perElementForces;
    for (const auto &vertexFaceCollision : vertexFaceCollisions) {
      const auto &collisionVertices = vertexFaceCollision.faceVertices;

      const Vec<Real> &v0 =
          this->strandMesh->der->vertices.row(vertexFaceCollision.vertexIndex);
      const Vec3<Real> &v1 = collisionVertices.at(0);
      const Vec3<Real> &v2 = collisionVertices.at(1);
      const Vec3<Real> &v3 = collisionVertices.at(2);

      Vec12<Real> x;
      x.segment<3>(0) = v0;
      x.segment<3>(3) = v1;
      x.segment<3>(6) = v2;
      x.segment<3>(9) = v3;

      const Vec12<Real> force =
          -TriangleArea(v1, v2, v3) * vertexFaceSqrtCollision->Gradient(x);

      R.segment<3>(3 * vertexFaceCollision.vertexIndex) += force.segment<3>(0);
    }

// If in debug, print out R norm to get an idea of the intensity
#ifndef NDEBUG
    std::cout << "R norm: " << R.norm() << std::endl;
    std::cout << "Velocity norm: " << this->velocity.norm() << std::endl;
#endif

    // Compute the update
    Vec<Real> u = (this->dt * this->dt) * this->strandMesh->der->mInv *
                      (R + this->externalForce) +
                  this->dt * this->velocity;

    // Pin the ends
    for (int ii = 0; ii < this->strandMesh->pinned.rows(); ++ii) {
      if (this->strandMesh->pinned(ii) == 1) {
        u.segment<3>(3 * ii) = Vec3<Real>::Zero();
      }
    }

    this->velocity = u / this->dt;

    // Sparse identity matrix
    Real coeff = bump ? 0.45 : 0.75;
    SparseMat<Real> filter =
        ConstructDiagonalSparseMatrix(
            Vec<Real>::Ones(this->strandMesh->DOFs()).eval()) -
        this->dt * coeff * this->strandMesh->der->mInv;
    this->velocity = filter * this->velocity;

    Vec<Real> positions = this->strandMesh->Positions();
    u += positions;
    this->strandMesh->SetPositions(u);

    this->strandMesh->der->thetas(0) +=
        this->strandMesh->der->leftAngularVelocity * this->dt;
    this->strandMesh->der->thetas(this->strandMesh->der->thetas.rows() - 1) +=
        this->strandMesh->der->rightAngularVelocity * this->dt;

    this->strandMesh->der->UpdateLengths();
    this->strandMesh->der->Computekbs();
    this->strandMesh->der->UpdateBishopFrames();
    //    this->strandMesh->der->UpdateQuasistaticMaterialFrame();
    this->strandMesh->der->UpdateMaterialFrames();
    this->strandMesh->der->UpdateMaterialCurvatures();
    this->strandMesh->der->UpdateKbGradients();
    this->strandMesh->der->UpdateHolonomyGradient();
  }
};