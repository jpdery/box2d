// requires 
var b2Vec2 = require('../../common/math/vec2.js'); 
var b2Mat22 = require('../../common/math/mat22.js'); 
var b2Math = require('../../common/math/math.js'); 
var b2FrictionJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2FrictionJoint.prototype, b2Joint.prototype)
b2FrictionJoint.prototype._super = b2Joint.prototype;
b2FrictionJoint.prototype.__constructor = function (def) {
		this._super.__constructor.apply(this, [def]);
		
		this.m_localAnchorA.SetV(def.localAnchorA);
		this.m_localAnchorB.SetV(def.localAnchorB);
		
		this.m_linearMass.SetZero();
		this.m_angularMass = 0.0;
		
		this.m_linearImpulse.SetZero();
		this.m_angularImpulse = 0.0;
		
		this.m_maxForce = def.maxForce;
		this.m_maxTorque = def.maxTorque;
	}
b2FrictionJoint.prototype.__varz = function(){
this.m_localAnchorA =  new b2Vec2();
this.m_localAnchorB =  new b2Vec2();
this.m_linearImpulse =  new b2Vec2();
this.m_linearMass =  new b2Mat22();
}
// static methods
// static attributes
// methods
b2FrictionJoint.prototype.InitVelocityConstraints = function (step) {
		var tMat;
		var tX;
		
		var bA = this.m_bodyA;
		var bB= this.m_bodyB;

		
		
		tMat = bA.m_xf.R;
		var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		
		tMat = bB.m_xf.R;
		var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;

		
		
		

		
		
		
		

		var mA = bA.m_invMass
		var mB = bB.m_invMass;
		var iA = bA.m_invI
		var iB = bB.m_invI;

		var K = new b2Mat22();
		K.col1.x = mA + mB;	K.col2.x = 0.0;
		K.col1.y = 0.0;		K.col2.y = mA + mB;

		K.col1.x+= iA * rAY * rAY;	K.col2.x+= -iA * rAX * rAY;
		K.col1.y+= -iA * rAX * rAY;	K.col2.y+= iA * rAX * rAX;

		K.col1.x+= iB * rBY * rBY;	K.col2.x+= -iB * rBX * rBY;
		K.col1.y+= -iB * rBX * rBY;	K.col2.y+= iB * rBX * rBX;

		K.GetInverse(this.m_linearMass);

		this.m_angularMass = iA + iB;
		if (this.m_angularMass > 0.0)
		{
			this.m_angularMass = 1.0 / this.m_angularMass;
		}

		if (step.warmStarting)
		{
			
			this.m_linearImpulse.x *= step.dtRatio;
			this.m_linearImpulse.y *= step.dtRatio;
			this.m_angularImpulse *= step.dtRatio;

			var P = this.m_linearImpulse;

			bA.m_linearVelocity.x -= mA * P.x;
			bA.m_linearVelocity.y -= mA * P.y;
			bA.m_angularVelocity -= iA * (rAX * P.y - rAY * P.x + this.m_angularImpulse);

			bB.m_linearVelocity.x += mB * P.x;
			bB.m_linearVelocity.y += mB * P.y;
			bB.m_angularVelocity += iB * (rBX * P.y - rBY * P.x + this.m_angularImpulse);
		}
		else
		{
			this.m_linearImpulse.SetZero();
			this.m_angularImpulse = 0.0;
		}

	}
b2FrictionJoint.prototype.SolveVelocityConstraints = function (step) {
		
		var tMat;
		var tX;

		var bA = this.m_bodyA;
		var bB= this.m_bodyB;

		var vA = bA.m_linearVelocity;
		var wA = bA.m_angularVelocity;
		var vB = bB.m_linearVelocity;
		var wB = bB.m_angularVelocity;

		var mA = bA.m_invMass
		var mB = bB.m_invMass;
		var iA = bA.m_invI
		var iB = bB.m_invI;

		
		tMat = bA.m_xf.R;
		var rAX = this.m_localAnchorA.x - bA.m_sweep.localCenter.x;
		var rAY = this.m_localAnchorA.y - bA.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rAX + tMat.col2.x * rAY);
		rAY = (tMat.col1.y * rAX + tMat.col2.y * rAY);
		rAX = tX;
		
		tMat = bB.m_xf.R;
		var rBX = this.m_localAnchorB.x - bB.m_sweep.localCenter.x;
		var rBY = this.m_localAnchorB.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * rBX + tMat.col2.x * rBY);
		rBY = (tMat.col1.y * rBX + tMat.col2.y * rBY);
		rBX = tX;
		
		var maxImpulse;

		
		{
			var Cdot = wB - wA;
			var impulse = -this.m_angularMass * Cdot;

			var oldImpulse = this.m_angularImpulse;
			maxImpulse = step.dt * this.m_maxTorque;
			this.m_angularImpulse = b2Math.Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.m_angularImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		
		{
			
			var CdotX = vB.x - wB * rBY - vA.x + wA * rAY;
			var CdotY = vB.y + wB * rBX - vA.y - wA * rAX;

			var impulseV = b2Math.MulMV(this.m_linearMass, new b2Vec2(-CdotX, -CdotY));
			var oldImpulseV = this.m_linearImpulse.Copy();
			
			this.m_linearImpulse.Add(impulseV);

			maxImpulse = step.dt * this.m_maxForce;

			if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
			{
				this.m_linearImpulse.Normalize();
				this.m_linearImpulse.Multiply(maxImpulse);
			}

			impulseV = b2Math.SubtractVV(this.m_linearImpulse, oldImpulseV);

			vA.x -= mA * impulseV.x;
			vA.y -= mA * impulseV.y;
			wA -= iA * (rAX * impulseV.y - rAY * impulseV.x);

			vB.x += mB * impulseV.x;
			vB.y += mB * impulseV.y;
			wB += iB * (rBX * impulseV.y - rBY * impulseV.x);
		}

		
		
		bA.m_angularVelocity = wA;
		
		bB.m_angularVelocity = wB;

	}
b2FrictionJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		
		
		return true;
		
	}
b2FrictionJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	}
b2FrictionJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	}
b2FrictionJoint.prototype.GetReactionForce = function (inv_dt) {
		return new b2Vec2(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
	}
b2FrictionJoint.prototype.GetReactionTorque = function (inv_dt) {
		
		return inv_dt * this.m_angularImpulse;
	}
b2FrictionJoint.prototype.SetMaxForce = function (force) {
		this.m_maxForce = force;
	}
b2FrictionJoint.prototype.GetMaxForce = function () {
		return this.m_maxForce;
	}
b2FrictionJoint.prototype.SetMaxTorque = function (torque) {
		this.m_maxTorque = torque;
	}
b2FrictionJoint.prototype.GetMaxTorque = function () {
		return this.m_maxTorque;
	}
// attributes
b2FrictionJoint.prototype.m_localAnchorA =  new b2Vec2();
b2FrictionJoint.prototype.m_localAnchorB =  new b2Vec2();
b2FrictionJoint.prototype.m_linearImpulse =  new b2Vec2();
b2FrictionJoint.prototype.m_angularImpulse =  null;
b2FrictionJoint.prototype.m_maxForce =  null;
b2FrictionJoint.prototype.m_maxTorque =  null;
b2FrictionJoint.prototype.m_linearMass =  new b2Mat22();
b2FrictionJoint.prototype.m_angularMass =  null;// aliases  
b2FrictionJoint.prototype.initVelocityConstraints = b2FrictionJoint.prototype.InitVelocityConstraints; 
b2FrictionJoint.prototype.solveVelocityConstraints = b2FrictionJoint.prototype.SolveVelocityConstraints; 
b2FrictionJoint.prototype.solvePositionConstraints = b2FrictionJoint.prototype.SolvePositionConstraints; 
b2FrictionJoint.prototype.getAnchorA = b2FrictionJoint.prototype.GetAnchorA; 
b2FrictionJoint.prototype.getAnchorB = b2FrictionJoint.prototype.GetAnchorB; 
b2FrictionJoint.prototype.getReactionForce = b2FrictionJoint.prototype.GetReactionForce; 
b2FrictionJoint.prototype.getReactionTorque = b2FrictionJoint.prototype.GetReactionTorque; 
b2FrictionJoint.prototype.setMaxForce = b2FrictionJoint.prototype.SetMaxForce; 
b2FrictionJoint.prototype.getMaxForce = b2FrictionJoint.prototype.GetMaxForce; 
b2FrictionJoint.prototype.setMaxTorque = b2FrictionJoint.prototype.SetMaxTorque; 
b2FrictionJoint.prototype.getMaxTorque = b2FrictionJoint.prototype.GetMaxTorque; 
// exports  
module.exports =  b2FrictionJoint;
