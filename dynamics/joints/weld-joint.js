// requires 
var b2Mat33 = require('../../common/math/mat33.js'); 
var b2Vec2 = require('../../common/math/vec2.js'); 
var b2Vec3 = require('../../common/math/vec3.js'); 
var b2Settings = require('../../common/settings.js'); 
var b2Math = require('../../common/math/math.js'); 
var b2WeldJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2WeldJoint.prototype, b2Joint.prototype)
b2WeldJoint.prototype._super = b2Joint.prototype;
b2WeldJoint.prototype.__constructor = function (def) {
		this._super.__constructor.apply(this, [def]);
		
		this.m_localAnchorA.SetV(def.localAnchorA);
		this.m_localAnchorB.SetV(def.localAnchorB);
		this.m_referenceAngle = def.referenceAngle;

		this.m_impulse.SetZero();
		this.m_mass = new b2Mat33();
	}
b2WeldJoint.prototype.__varz = function(){
this.m_localAnchorA =  new b2Vec2();
this.m_localAnchorB =  new b2Vec2();
this.m_impulse =  new b2Vec3();
this.m_mass =  new b2Mat33();
}
// static methods
// static attributes
// methods
b2WeldJoint.prototype.InitVelocityConstraints = function (step) {
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
		
		this.m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB;
		this.m_mass.col2.x = -rAY * rAX * iA - rBY * rBX * iB;
		this.m_mass.col3.x = -rAY * iA - rBY * iB;
		this.m_mass.col1.y = this.m_mass.col2.x;
		this.m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB;
		this.m_mass.col3.y = rAX * iA + rBX * iB;
		this.m_mass.col1.z = this.m_mass.col3.x;
		this.m_mass.col2.z = this.m_mass.col3.y;
		this.m_mass.col3.z = iA + iB;
		
		if (step.warmStarting)
		{
			
			this.m_impulse.x *= step.dtRatio;
			this.m_impulse.y *= step.dtRatio;
			this.m_impulse.z *= step.dtRatio;

			bA.m_linearVelocity.x -= mA * this.m_impulse.x;
			bA.m_linearVelocity.y -= mA * this.m_impulse.y;
			bA.m_angularVelocity -= iA * (rAX * this.m_impulse.y - rAY * this.m_impulse.x + this.m_impulse.z);

			bB.m_linearVelocity.x += mB * this.m_impulse.x;
			bB.m_linearVelocity.y += mB * this.m_impulse.y;
			bB.m_angularVelocity += iB * (rBX * this.m_impulse.y - rBY * this.m_impulse.x + this.m_impulse.z);
		}
		else
		{
			this.m_impulse.SetZero();
		}

	}
b2WeldJoint.prototype.SolveVelocityConstraints = function (step) {
		
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

		
		
		var Cdot1X = vB.x - wB * rBY - vA.x + wA * rAY;
		var Cdot1Y = vB.y + wB * rBX - vA.y - wA * rAX;
		var Cdot2 = wB - wA;
		var impulse = new b2Vec3();
		this.m_mass.Solve33(impulse, -Cdot1X, -Cdot1Y, -Cdot2);
		
		this.m_impulse.Add(impulse);
		
		vA.x -= mA * impulse.x;
		vA.y -= mA * impulse.y;
		wA -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z);

		vB.x += mB * impulse.x;
		vB.y += mB * impulse.y;
		wB += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z);

		
		
		bA.m_angularVelocity = wA;
		
		bB.m_angularVelocity = wB;

	}
b2WeldJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		
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
		
		
		var C1X = bB.m_sweep.c.x + rBX - bA.m_sweep.c.x - rAX;
		var C1Y = bB.m_sweep.c.y + rBY - bA.m_sweep.c.y - rAY;
		var C2 = bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;

		
		var k_allowedStretch = 10.0 * b2Settings.b2_linearSlop;
		var positionError = Math.sqrt(C1X * C1X + C1Y * C1Y);
		var angularError = b2Math.Abs(C2);
		if (positionError > k_allowedStretch)
		{
			iA *= 1.0;
			iB *= 1.0;
		}
		
		this.m_mass.col1.x = mA + mB + rAY * rAY * iA + rBY * rBY * iB;
		this.m_mass.col2.x = -rAY * rAX * iA - rBY * rBX * iB;
		this.m_mass.col3.x = -rAY * iA - rBY * iB;
		this.m_mass.col1.y = this.m_mass.col2.x;
		this.m_mass.col2.y = mA + mB + rAX * rAX * iA + rBX * rBX * iB;
		this.m_mass.col3.y = rAX * iA + rBX * iB;
		this.m_mass.col1.z = this.m_mass.col3.x;
		this.m_mass.col2.z = this.m_mass.col3.y;
		this.m_mass.col3.z = iA + iB;
		
		var impulse = new b2Vec3();
		this.m_mass.Solve33(impulse, -C1X, -C1Y, -C2);
		

		bA.m_sweep.c.x -= mA * impulse.x;
		bA.m_sweep.c.y -= mA * impulse.y;
		bA.m_sweep.a -= iA * (rAX * impulse.y - rAY * impulse.x + impulse.z);

		bB.m_sweep.c.x += mB * impulse.x;
		bB.m_sweep.c.y += mB * impulse.y;
		bB.m_sweep.a += iB * (rBX * impulse.y - rBY * impulse.x + impulse.z);

		bA.SynchronizeTransform();
		bB.SynchronizeTransform();

		return positionError <= b2Settings.b2_linearSlop && angularError <= b2Settings.b2_angularSlop;

	}
b2WeldJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	}
b2WeldJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	}
b2WeldJoint.prototype.GetReactionForce = function (inv_dt) {
		return new b2Vec2(inv_dt * this.m_impulse.x, inv_dt * this.m_impulse.y);
	}
b2WeldJoint.prototype.GetReactionTorque = function (inv_dt) {
		return inv_dt * this.m_impulse.z;
	}
// attributes
b2WeldJoint.prototype.m_localAnchorA =  new b2Vec2();
b2WeldJoint.prototype.m_localAnchorB =  new b2Vec2();
b2WeldJoint.prototype.m_referenceAngle =  null;
b2WeldJoint.prototype.m_impulse =  new b2Vec3();
b2WeldJoint.prototype.m_mass =  new b2Mat33();// aliases  
b2WeldJoint.prototype.initVelocityConstraints = b2WeldJoint.prototype.InitVelocityConstraints; 
b2WeldJoint.prototype.solveVelocityConstraints = b2WeldJoint.prototype.SolveVelocityConstraints; 
b2WeldJoint.prototype.solvePositionConstraints = b2WeldJoint.prototype.SolvePositionConstraints; 
b2WeldJoint.prototype.getAnchorA = b2WeldJoint.prototype.GetAnchorA; 
b2WeldJoint.prototype.getAnchorB = b2WeldJoint.prototype.GetAnchorB; 
b2WeldJoint.prototype.getReactionForce = b2WeldJoint.prototype.GetReactionForce; 
b2WeldJoint.prototype.getReactionTorque = b2WeldJoint.prototype.GetReactionTorque; 
// exports  
module.exports =  b2WeldJoint;
