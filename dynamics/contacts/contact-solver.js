// requires 
var b2TimeStep = require('../../dynamics/time-step.js'); 
var b2WorldManifold = require('../../collision/world-manifold.js'); 
var b2PositionSolverManifold = require('./position-solver-manifold.js'); 
var b2ContactConstraint = require('./contact-constraint.js'); 
var b2Settings = require('../../common/settings.js'); 
var b2Math = require('../../common/math/math.js'); 
var b2ContactSolver = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactSolver.prototype.__constructor = function () {
	}
b2ContactSolver.prototype.__varz = function(){
this.m_step =  new b2TimeStep();
this.m_constraints =  new Array ();
}
// static methods
// static attributes
b2ContactSolver.s_worldManifold =  new b2WorldManifold();
b2ContactSolver.s_psm =  new b2PositionSolverManifold();
// methods
b2ContactSolver.prototype.Initialize = function (step, contacts, contactCount, allocator) {
		var contact;
		
		this.m_step.Set(step)
		
		this.m_allocator = allocator;
		
		var i = 0;
		var tVec;
		var tMat;
		
		this.m_constraintCount = contactCount;

		
		while (this.m_constraints.length < this.m_constraintCount)
		{
			this.m_constraints[this.m_constraints.length] = new b2ContactConstraint();
		}
		
		for (i = 0; i < contactCount; ++i)
		{
			contact = contacts[i];
			var fixtureA = contact.m_fixtureA;
			var fixtureB = contact.m_fixtureB;
			var shapeA = fixtureA.m_shape;
			var shapeB = fixtureB.m_shape;
			var radiusA = shapeA.m_radius;
			var radiusB = shapeB.m_radius;
			var bodyA = fixtureA.m_body;
			var bodyB = fixtureB.m_body;
			var manifold = contact.GetManifold();
			
			var friction = b2Settings.b2MixFriction(fixtureA.GetFriction(), fixtureB.GetFriction());
			var restitution = b2Settings.b2MixRestitution(fixtureA.GetRestitution(), fixtureB.GetRestitution());
			
			
			var vAX = bodyA.m_linearVelocity.x;
			var vAY = bodyA.m_linearVelocity.y;
			
			var vBX = bodyB.m_linearVelocity.x;
			var vBY = bodyB.m_linearVelocity.y;
			var wA = bodyA.m_angularVelocity;
			var wB = bodyB.m_angularVelocity;
			
			b2Settings.b2Assert(manifold.m_pointCount > 0);
			
			b2ContactSolver.s_worldManifold.Initialize(manifold, bodyA.m_xf, radiusA, bodyB.m_xf, radiusB);
			
			var normalX = b2ContactSolver.s_worldManifold.m_normal.x;
			var normalY = b2ContactSolver.s_worldManifold.m_normal.y;
			
			var cc = this.m_constraints[ i ];
			cc.bodyA = bodyA; 
			cc.bodyB = bodyB; 
			cc.manifold = manifold; 
			
			cc.normal.x = normalX;
			cc.normal.y = normalY;
			cc.pointCount = manifold.m_pointCount;
			cc.friction = friction;
			cc.restitution = restitution;
			
			cc.localPlaneNormal.x = manifold.m_localPlaneNormal.x;
			cc.localPlaneNormal.y = manifold.m_localPlaneNormal.y;
			cc.localPoint.x = manifold.m_localPoint.x;
			cc.localPoint.y = manifold.m_localPoint.y;
			cc.radius = radiusA + radiusB;
			cc.type = manifold.m_type;
			
			for (var k = 0; k < cc.pointCount; ++k)
			{
				var cp = manifold.m_points[ k ];
				var ccp = cc.points[ k ];
				
				ccp.normalImpulse = cp.m_normalImpulse;
				ccp.tangentImpulse = cp.m_tangentImpulse;
				
				ccp.localPoint.SetV(cp.m_localPoint);
				
				var rAX = ccp.rA.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyA.m_sweep.c.x;
				var rAY = ccp.rA.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyA.m_sweep.c.y;
				var rBX = ccp.rB.x = b2ContactSolver.s_worldManifold.m_points[k].x - bodyB.m_sweep.c.x;
				var rBY = ccp.rB.y = b2ContactSolver.s_worldManifold.m_points[k].y - bodyB.m_sweep.c.y;
				
				var rnA = rAX * normalY - rAY * normalX;
				var rnB = rBX * normalY - rBY * normalX;
				
				rnA *= rnA;
				rnB *= rnB;
				
				var kNormal = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rnA + bodyB.m_invI * rnB;
				
				ccp.normalMass = 1.0 / kNormal;
				
				var kEqualized = bodyA.m_mass * bodyA.m_invMass + bodyB.m_mass * bodyB.m_invMass;
				kEqualized += bodyA.m_mass * bodyA.m_invI * rnA + bodyB.m_mass * bodyB.m_invI * rnB;
				
				ccp.equalizedMass = 1.0 / kEqualized;
				
				
				var tangentX = normalY
				var tangentY = -normalX;
				
				
				var rtA = rAX*tangentY - rAY*tangentX;
				
				var rtB = rBX*tangentY - rBY*tangentX;
				
				rtA *= rtA;
				rtB *= rtB;
				
				var kTangent = bodyA.m_invMass + bodyB.m_invMass + bodyA.m_invI * rtA + bodyB.m_invI * rtB;
				
				ccp.tangentMass = 1.0 / kTangent;
				
				
				ccp.velocityBias = 0.0;
				
				var tX = vBX + (-wB*rBY) - vAX - (-wA*rAY);
				var tY = vBY + (wB*rBX) - vAY - (wA*rAX);
				
				var vRel = cc.normal.x*tX + cc.normal.y*tY;
				if (vRel < -b2Settings.b2_velocityThreshold)
				{
					ccp.velocityBias += -cc.restitution * vRel;
				}
			}
			
			
			if (cc.pointCount == 2)
			{
				var ccp1 = cc.points[0];
				var ccp2 = cc.points[1];
				
				var invMassA = bodyA.m_invMass;
				var invIA = bodyA.m_invI;
				var invMassB = bodyB.m_invMass;
				var invIB = bodyB.m_invI;
				
				
				
				
				
				var rn1A = ccp1.rA.x * normalY - ccp1.rA.y * normalX;
				var rn1B = ccp1.rB.x * normalY - ccp1.rB.y * normalX;
				var rn2A = ccp2.rA.x * normalY - ccp2.rA.y * normalX;
				var rn2B = ccp2.rB.x * normalY - ccp2.rB.y * normalX;
				
				var k11 = invMassA + invMassB + invIA * rn1A * rn1A + invIB * rn1B * rn1B;
				var k22 = invMassA + invMassB + invIA * rn2A * rn2A + invIB * rn2B * rn2B;
				var k12 = invMassA + invMassB + invIA * rn1A * rn2A + invIB * rn1B * rn2B;
				
				
				var k_maxConditionNumber = 100.0;
				if ( k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
				{
					
					cc.K.col1.Set(k11, k12);
					cc.K.col2.Set(k12, k22);
					cc.K.GetInverse(cc.normalMass);
				}
				else
				{
					
					
					cc.pointCount = 1;
				}
			}
		}
		
		
	}
b2ContactSolver.prototype.InitVelocityConstraints = function (step) {
		var tVec;
		var tVec2;
		var tMat;
		
		
		for (var i = 0; i < this.m_constraintCount; ++i)
		{
			var c = this.m_constraints[ i ];
			
			var bodyA = c.bodyA;
			var bodyB = c.bodyB;
			var invMassA = bodyA.m_invMass;
			var invIA = bodyA.m_invI;
			var invMassB = bodyB.m_invMass;
			var invIB = bodyB.m_invI;
			
			var normalX = c.normal.x;
			var normalY = c.normal.y;
			
			var tangentX = normalY;
			var tangentY = -normalX;
			
			var tX;
			
			var j = 0;
			var tCount = 0;
			if (step.warmStarting)
			{
				tCount = c.pointCount;
				for (j = 0; j < tCount; ++j)
				{
					var ccp = c.points[ j ];
					ccp.normalImpulse *= step.dtRatio;
					ccp.tangentImpulse *= step.dtRatio;
					
					var PX = ccp.normalImpulse * normalX + ccp.tangentImpulse * tangentX;
					var PY = ccp.normalImpulse * normalY + ccp.tangentImpulse * tangentY;
					
					
					bodyA.m_angularVelocity -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
					
					bodyA.m_linearVelocity.x -= invMassA * PX;
					bodyA.m_linearVelocity.y -= invMassA * PY;
					
					bodyB.m_angularVelocity += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
					
					bodyB.m_linearVelocity.x += invMassB * PX;
					bodyB.m_linearVelocity.y += invMassB * PY;
				}
			}
			else
			{
				tCount = c.pointCount;
				for (j = 0; j < tCount; ++j)
				{
					var ccp2 = c.points[ j ];
					ccp2.normalImpulse = 0.0;
					ccp2.tangentImpulse = 0.0;
				}
			}
		}
	}
b2ContactSolver.prototype.SolveVelocityConstraints = function () {
		var j = 0;
		var ccp;
		var rAX;
		var rAY;
		var rBX;
		var rBY;
		var dvX;
		var dvY;
		var vn;
		var vt;
		var lambda;
		var maxFriction;
		var newImpulse;
		var PX;
		var PY;
		var dX;
		var dY;
		var P1X;
		var P1Y;
		var P2X;
		var P2Y;
		
		var tMat;
		var tVec;
		
		for (var i = 0; i < this.m_constraintCount; ++i)
		{
			var c = this.m_constraints[ i ];
			var bodyA = c.bodyA;
			var bodyB = c.bodyB;
			var wA = bodyA.m_angularVelocity;
			var wB = bodyB.m_angularVelocity;
			var vA = bodyA.m_linearVelocity;
			var vB = bodyB.m_linearVelocity;
			
			var invMassA = bodyA.m_invMass;
			var invIA = bodyA.m_invI;
			var invMassB = bodyB.m_invMass;
			var invIB = bodyB.m_invI;
			
			var normalX = c.normal.x;
			var normalY = c.normal.y;
			
			var tangentX = normalY;
			var tangentY = -normalX;
			var friction = c.friction;
			
			var tX;
			
			
			
			for (j = 0; j < c.pointCount; j++)
			{
				ccp = c.points[j];
				
				
				
				dvX = vB.x - wB * ccp.rB.y - vA.x + wA * ccp.rA.y;
				dvY = vB.y + wB * ccp.rB.x - vA.y - wA * ccp.rA.x;
				
				
				vt = dvX * tangentX + dvY * tangentY;
				lambda = ccp.tangentMass * -vt;
				
				
				maxFriction = friction * ccp.normalImpulse;
				newImpulse = b2Math.Clamp(ccp.tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse-ccp.tangentImpulse;
				
				
				PX = lambda * tangentX;
				PY = lambda * tangentY;
				
				vA.x -= invMassA * PX;
				vA.y -= invMassA * PY;
				wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
				
				vB.x += invMassB * PX;
				vB.y += invMassB * PY;
				wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
				
				ccp.tangentImpulse = newImpulse;
			}
			
			
			var tCount = c.pointCount;
			if (c.pointCount == 1)
			{
				ccp = c.points[ 0 ];
				
				
				
				dvX = vB.x + (-wB * ccp.rB.y) - vA.x - (-wA * ccp.rA.y);
				dvY = vB.y + (wB * ccp.rB.x) - vA.y - (wA * ccp.rA.x);
				
				
				
				vn = dvX * normalX + dvY * normalY;
				lambda = -ccp.normalMass * (vn - ccp.velocityBias);
				
				
				
				newImpulse = ccp.normalImpulse + lambda;
				newImpulse = newImpulse > 0 ? newImpulse : 0.0;
				lambda = newImpulse - ccp.normalImpulse;
				
				
				
				PX = lambda * normalX;
				PY = lambda * normalY;
				
				
				vA.x -= invMassA * PX;
				vA.y -= invMassA * PY;
				wA -= invIA * (ccp.rA.x * PY - ccp.rA.y * PX);
				
				
				vB.x += invMassB * PX;
				vB.y += invMassB * PY;
				wB += invIB * (ccp.rB.x * PY - ccp.rB.y * PX);
				
				ccp.normalImpulse = newImpulse;
			}
			else
			{
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				
				var cp1 = c.points[ 0 ];
				var cp2 = c.points[ 1 ];
				
				var aX = cp1.normalImpulse;
				var aY = cp2.normalImpulse;
				
				
				
				
				var dv1X = vB.x - wB * cp1.rB.y - vA.x + wA * cp1.rA.y;
				var dv1Y = vB.y + wB * cp1.rB.x - vA.y - wA * cp1.rA.x;
				
				var dv2X = vB.x - wB * cp2.rB.y - vA.x + wA * cp2.rA.y;
				var dv2Y = vB.y + wB * cp2.rB.x - vA.y - wA * cp2.rA.x;
				
				
				
				var vn1 = dv1X * normalX + dv1Y * normalY;
				
				var vn2 = dv2X * normalX + dv2Y * normalY;
				
				var bX = vn1 - cp1.velocityBias;
				var bY = vn2 - cp2.velocityBias;
				
				
				tMat = c.K;
				bX -= tMat.col1.x * aX + tMat.col2.x * aY;
				bY -= tMat.col1.y * aX + tMat.col2.y * aY;
				
				var k_errorTol = 0.001;
				for (;; )
				{
					
					
					
					
					
					
					
					
					
					
					
					tMat = c.normalMass;
					var xX = - (tMat.col1.x * bX + tMat.col2.x * bY);
					var xY = - (tMat.col1.y * bX + tMat.col2.y * bY);
					
					if (xX >= 0.0 && xY >= 0.0) {
						
						
						dX = xX - aX;
						dY = xY - aY;
						
						
						
						P1X = dX * normalX;
						P1Y = dX * normalY;
						
						P2X = dY * normalX;
						P2Y = dY * normalY;
						
						
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						
						wA -= invIA * ( cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
						
						
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						
						wB += invIB * ( cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
						
						
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
						break;
					}
					
					
					
					
					
					
					
					
					xX = - cp1.normalMass * bX;
					xY = 0.0;
					vn1 = 0.0;
					vn2 = c.K.col1.y * xX + bY;
					
					if (xX >= 0.0 && vn2 >= 0.0)
					{
						
						
						dX = xX - aX;
						dY = xY - aY;
						
						
						
						P1X = dX * normalX;
						P1Y = dX * normalY;
						
						P2X = dY * normalX;
						P2Y = dY * normalY;
						
						
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						
						wA -= invIA * ( cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
						
						
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						
						wB += invIB * ( cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
						
						
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
						break;
					}
					
					
					
					
					
					
					
					
					xX = 0.0;
					xY = -cp2.normalMass * bY;
					vn1 = c.K.col2.x * xY + bX;
					vn2 = 0.0;
					if (xY >= 0.0 && vn1 >= 0.0)
					{
						
						
						dX = xX - aX;
						dY = xY - aY;
						
						
						
						P1X = dX * normalX;
						P1Y = dX * normalY;
						
						P2X = dY * normalX;
						P2Y = dY * normalY;
						
						
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						
						wA -= invIA * ( cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
						
						
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						
						wB += invIB * ( cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
						
						
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
						break;
					}
					
					
					
					
					
					
					
					xX = 0.0;
					xY = 0.0;
					vn1 = bX;
					vn2 = bY;
					
					if (vn1 >= 0.0 && vn2 >= 0.0 ) {
						
						
						dX = xX - aX;
						dY = xY - aY;
						
						
						
						P1X = dX * normalX;
						P1Y = dX * normalY;
						
						P2X = dY * normalX;
						P2Y = dY * normalY;
						
						
						vA.x -= invMassA * (P1X + P2X);
						vA.y -= invMassA * (P1Y + P2Y);
						
						wA -= invIA * ( cp1.rA.x * P1Y - cp1.rA.y * P1X + cp2.rA.x * P2Y - cp2.rA.y * P2X);
						
						
						vB.x += invMassB * (P1X + P2X);
						vB.y += invMassB * (P1Y + P2Y);
						
						wB += invIB * ( cp1.rB.x * P1Y - cp1.rB.y * P1X + cp2.rB.x * P2Y - cp2.rB.y * P2X);
						
						
						cp1.normalImpulse = xX;
						cp2.normalImpulse = xY;
						
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
						break;
					}
					
					
					break;
				}
			}
			
			
			
			
			
			
			
			bodyA.m_angularVelocity = wA;
			bodyB.m_angularVelocity = wB;
		}
	}
b2ContactSolver.prototype.FinalizeVelocityConstraints = function () {
		for (var i = 0; i < this.m_constraintCount; ++i)
		{
			var c = this.m_constraints[ i ];
			var m = c.manifold;
			
			for (var j = 0; j < c.pointCount; ++j)
			{
				var point1 = m.m_points[j];
				var point2 = c.points[j];
				point1.m_normalImpulse = point2.normalImpulse;
				point1.m_tangentImpulse = point2.tangentImpulse;
			}
		}
	}
b2ContactSolver.prototype.SolvePositionConstraints = function (baumgarte) {
		var minSeparation = 0.0;
		
		for (var i = 0; i < this.m_constraintCount; i++)
		{
			var c = this.m_constraints[i];
			var bodyA = c.bodyA;
			var bodyB = c.bodyB;
			
			var invMassA = bodyA.m_mass * bodyA.m_invMass;
			var invIA = bodyA.m_mass * bodyA.m_invI;
			var invMassB = bodyB.m_mass * bodyB.m_invMass;
			var invIB = bodyB.m_mass * bodyB.m_invI;
			
			
			b2ContactSolver.s_psm.Initialize(c);
			var normal = b2ContactSolver.s_psm.m_normal;
			
			
			for (var j = 0; j < c.pointCount; j++)
			{
				var ccp = c.points[j];
				
				var point = b2ContactSolver.s_psm.m_points[j];
				var separation = b2ContactSolver.s_psm.m_separations[j];
				
				var rAX = point.x - bodyA.m_sweep.c.x;
				var rAY = point.y - bodyA.m_sweep.c.y;
				var rBX = point.x - bodyB.m_sweep.c.x;
				var rBY = point.y - bodyB.m_sweep.c.y;
				
				
				minSeparation = minSeparation < separation?minSeparation:separation;
				
				
				var C = b2Math.Clamp(baumgarte * (separation + b2Settings.b2_linearSlop), -b2Settings.b2_maxLinearCorrection, 0.0);
				
				
				var impulse = -ccp.equalizedMass * C;
				
				var PX = impulse * normal.x;
				var PY = impulse * normal.y;
				
				
				bodyA.m_sweep.c.x -= invMassA * PX;
				bodyA.m_sweep.c.y -= invMassA * PY;
				
				bodyA.m_sweep.a -= invIA * (rAX * PY - rAY * PX);
				bodyA.SynchronizeTransform();
				
				
				bodyB.m_sweep.c.x += invMassB * PX;
				bodyB.m_sweep.c.y += invMassB * PY;
				
				bodyB.m_sweep.a += invIB * (rBX * PY - rBY * PX);
				bodyB.SynchronizeTransform();
			}
		}
		
		
		
		return minSeparation > -1.5 * b2Settings.b2_linearSlop;
	}
// attributes
b2ContactSolver.prototype.m_step =  new b2TimeStep();
b2ContactSolver.prototype.m_allocator =  null;
b2ContactSolver.prototype.m_constraints =  new Array ();
b2ContactSolver.prototype.m_constraintCount =  0;// aliases  
b2ContactSolver.prototype.initialize = b2ContactSolver.prototype.Initialize; 
b2ContactSolver.prototype.initVelocityConstraints = b2ContactSolver.prototype.InitVelocityConstraints; 
b2ContactSolver.prototype.solveVelocityConstraints = b2ContactSolver.prototype.SolveVelocityConstraints; 
b2ContactSolver.prototype.finalizeVelocityConstraints = b2ContactSolver.prototype.FinalizeVelocityConstraints; 
b2ContactSolver.prototype.solvePositionConstraints = b2ContactSolver.prototype.SolvePositionConstraints; 
// exports  
module.exports =  b2ContactSolver;
