// requires 
var b2Vec2 = require('../../common/math/vec2.js'); 
var b2Settings = require('../../common/settings.js'); 
var b2Math = require('../../common/math/math.js'); 
var b2PulleyJoint = function() {
b2Joint.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2PulleyJoint.prototype, b2Joint.prototype)
b2PulleyJoint.prototype._super = b2Joint.prototype;
b2PulleyJoint.prototype.__constructor = function (def) {
		
		
		this._super.__constructor.apply(this, [def]);
		
		var tMat;
		var tX;
		var tY;
		
		this.m_ground = this.m_bodyA.m_world.m_groundBody;
		
		this.m_groundAnchor1.x = def.groundAnchorA.x - this.m_ground.m_xf.position.x;
		this.m_groundAnchor1.y = def.groundAnchorA.y - this.m_ground.m_xf.position.y;
		
		this.m_groundAnchor2.x = def.groundAnchorB.x - this.m_ground.m_xf.position.x;
		this.m_groundAnchor2.y = def.groundAnchorB.y - this.m_ground.m_xf.position.y;
		
		this.m_localAnchor1.SetV(def.localAnchorA);
		
		this.m_localAnchor2.SetV(def.localAnchorB);
		
		
		this.m_ratio = def.ratio;
		
		this.m_constant = def.lengthA + this.m_ratio * def.lengthB;
		
		this.m_maxLength1 = b2Math.Min(def.maxLengthA, this.m_constant - this.m_ratio * b2PulleyJoint.b2_minPulleyLength);
		this.m_maxLength2 = b2Math.Min(def.maxLengthB, (this.m_constant - b2PulleyJoint.b2_minPulleyLength) / this.m_ratio);
		
		this.m_impulse = 0.0;
		this.m_limitImpulse1 = 0.0;
		this.m_limitImpulse2 = 0.0;
		
	}
b2PulleyJoint.prototype.__varz = function(){
this.m_groundAnchor1 =  new b2Vec2();
this.m_groundAnchor2 =  new b2Vec2();
this.m_localAnchor1 =  new b2Vec2();
this.m_localAnchor2 =  new b2Vec2();
this.m_u1 =  new b2Vec2();
this.m_u2 =  new b2Vec2();
}
// static methods
// static attributes
b2PulleyJoint.b2_minPulleyLength =  2.0;
// methods
b2PulleyJoint.prototype.InitVelocityConstraints = function (step) {
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		
		var tMat;
		
		
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		
		
		var p1X = bA.m_sweep.c.x + r1X;
		var p1Y = bA.m_sweep.c.y + r1Y;
		
		var p2X = bB.m_sweep.c.x + r2X;
		var p2Y = bB.m_sweep.c.y + r2Y;
		
		
		var s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		var s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		
		var s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		var s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		
		
		
		this.m_u1.Set(p1X - s1X, p1Y - s1Y);
		
		this.m_u2.Set(p2X - s2X, p2Y - s2Y);
		
		var length1 = this.m_u1.Length();
		var length2 = this.m_u2.Length();
		
		if (length1 > b2Settings.b2_linearSlop)
		{
			
			this.m_u1.Multiply(1.0 / length1);
		}
		else
		{
			this.m_u1.SetZero();
		}
		
		if (length2 > b2Settings.b2_linearSlop)
		{
			
			this.m_u2.Multiply(1.0 / length2);
		}
		else
		{
			this.m_u2.SetZero();
		}
		
		var C = this.m_constant - length1 - this.m_ratio * length2;
		if (C > 0.0)
		{
			this.m_state = b2Joint.e_inactiveLimit;
			this.m_impulse = 0.0;
		}
		else
		{
			this.m_state = b2Joint.e_atUpperLimit;
		}
		
		if (length1 < this.m_maxLength1)
		{
			this.m_limitState1 = b2Joint.e_inactiveLimit;
			this.m_limitImpulse1 = 0.0;
		}
		else
		{
			this.m_limitState1 = b2Joint.e_atUpperLimit;
		}
		
		if (length2 < this.m_maxLength2)
		{
			this.m_limitState2 = b2Joint.e_inactiveLimit;
			this.m_limitImpulse2 = 0.0;
		}
		else
		{
			this.m_limitState2 = b2Joint.e_atUpperLimit;
		}
		
		
		
		var cr1u1 = r1X * this.m_u1.y - r1Y * this.m_u1.x;
		
		var cr2u2 = r2X * this.m_u2.y - r2Y * this.m_u2.x;
		
		this.m_limitMass1 = bA.m_invMass + bA.m_invI * cr1u1 * cr1u1;
		this.m_limitMass2 = bB.m_invMass + bB.m_invI * cr2u2 * cr2u2;
		this.m_pulleyMass = this.m_limitMass1 + this.m_ratio * this.m_ratio * this.m_limitMass2;
		
		
		
		this.m_limitMass1 = 1.0 / this.m_limitMass1;
		this.m_limitMass2 = 1.0 / this.m_limitMass2;
		this.m_pulleyMass = 1.0 / this.m_pulleyMass;
		
		if (step.warmStarting)
		{
			
			this.m_impulse *= step.dtRatio;
			this.m_limitImpulse1 *= step.dtRatio;
			this.m_limitImpulse2 *= step.dtRatio;
			
			
			
			var P1X = (-this.m_impulse - this.m_limitImpulse1) * this.m_u1.x;
			var P1Y = (-this.m_impulse - this.m_limitImpulse1) * this.m_u1.y;
			
			var P2X = (-this.m_ratio * this.m_impulse - this.m_limitImpulse2) * this.m_u2.x;
			var P2Y = (-this.m_ratio * this.m_impulse - this.m_limitImpulse2) * this.m_u2.y;
			
			bA.m_linearVelocity.x += bA.m_invMass * P1X;
			bA.m_linearVelocity.y += bA.m_invMass * P1Y;
			
			bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
			
			bB.m_linearVelocity.x += bB.m_invMass * P2X;
			bB.m_linearVelocity.y += bB.m_invMass * P2Y;
			
			bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
		}
		else
		{
			this.m_impulse = 0.0;
			this.m_limitImpulse1 = 0.0;
			this.m_limitImpulse2 = 0.0;
		}
	}
b2PulleyJoint.prototype.SolveVelocityConstraints = function (step) {
		
		
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		
		var tMat;
		
		
		tMat = bA.m_xf.R;
		var r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
		var r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
		var tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
		r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
		r1X = tX;
		
		tMat = bB.m_xf.R;
		var r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
		var r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
		tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
		r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
		r2X = tX;
		
		
		var v1X;
		var v1Y;
		var v2X;
		var v2Y;
		var P1X;
		var P1Y;
		var P2X;
		var P2Y;
		var Cdot;
		var impulse;
		var oldImpulse;
		
		if (this.m_state == b2Joint.e_atUpperLimit)
		{
			
			v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y);
			v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
			
			v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y);
			v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
			
			
			Cdot = -(this.m_u1.x * v1X + this.m_u1.y * v1Y) - this.m_ratio * (this.m_u2.x * v2X + this.m_u2.y * v2Y);
			impulse = this.m_pulleyMass * (-Cdot);
			oldImpulse = this.m_impulse;
			this.m_impulse = b2Math.Max(0.0, this.m_impulse + impulse);
			impulse = this.m_impulse - oldImpulse;
			
			
			P1X = -impulse * this.m_u1.x;
			P1Y = -impulse * this.m_u1.y;
			
			P2X = -this.m_ratio * impulse * this.m_u2.x;
			P2Y = -this.m_ratio * impulse * this.m_u2.y;
			
			bA.m_linearVelocity.x += bA.m_invMass * P1X;
			bA.m_linearVelocity.y += bA.m_invMass * P1Y;
			
			bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
			
			bB.m_linearVelocity.x += bB.m_invMass * P2X;
			bB.m_linearVelocity.y += bB.m_invMass * P2Y;
			
			bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
		}
		
		if (this.m_limitState1 == b2Joint.e_atUpperLimit)
		{
			
			v1X = bA.m_linearVelocity.x + (-bA.m_angularVelocity * r1Y);
			v1Y = bA.m_linearVelocity.y + (bA.m_angularVelocity * r1X);
			
			
			Cdot = -(this.m_u1.x * v1X + this.m_u1.y * v1Y);
			impulse = -this.m_limitMass1 * Cdot;
			oldImpulse = this.m_limitImpulse1;
			this.m_limitImpulse1 = b2Math.Max(0.0, this.m_limitImpulse1 + impulse);
			impulse = this.m_limitImpulse1 - oldImpulse;
			
			
			P1X = -impulse * this.m_u1.x;
			P1Y = -impulse * this.m_u1.y;
			
			bA.m_linearVelocity.x += bA.m_invMass * P1X;
			bA.m_linearVelocity.y += bA.m_invMass * P1Y;
			
			bA.m_angularVelocity += bA.m_invI * (r1X * P1Y - r1Y * P1X);
		}
		
		if (this.m_limitState2 == b2Joint.e_atUpperLimit)
		{
			
			v2X = bB.m_linearVelocity.x + (-bB.m_angularVelocity * r2Y);
			v2Y = bB.m_linearVelocity.y + (bB.m_angularVelocity * r2X);
			
			
			Cdot = -(this.m_u2.x * v2X + this.m_u2.y * v2Y);
			impulse = -this.m_limitMass2 * Cdot;
			oldImpulse = this.m_limitImpulse2;
			this.m_limitImpulse2 = b2Math.Max(0.0, this.m_limitImpulse2 + impulse);
			impulse = this.m_limitImpulse2 - oldImpulse;
			
			
			P2X = -impulse * this.m_u2.x;
			P2Y = -impulse * this.m_u2.y;
			
			bB.m_linearVelocity.x += bB.m_invMass * P2X;
			bB.m_linearVelocity.y += bB.m_invMass * P2Y;
			
			bB.m_angularVelocity += bB.m_invI * (r2X * P2Y - r2Y * P2X);
		}
	}
b2PulleyJoint.prototype.SolvePositionConstraints = function (baumgarte) {
		
		
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		
		var tMat;
		
		
		var s1X = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		var s1Y = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		
		var s2X = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		var s2Y = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		
		
		var r1X;
		var r1Y;
		var r2X;
		var r2Y;
		var p1X;
		var p1Y;
		var p2X;
		var p2Y;
		var length1;
		var length2;
		var C;
		var impulse;
		var oldImpulse;
		var oldLimitPositionImpulse;
		
		var tX;
		
		var linearError = 0.0;
		
		if (this.m_state == b2Joint.e_atUpperLimit)
		{
			
			tMat = bA.m_xf.R;
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			
			tMat = bB.m_xf.R;
			r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			
			
			p1X = bA.m_sweep.c.x + r1X;
			p1Y = bA.m_sweep.c.y + r1Y;
			
			p2X = bB.m_sweep.c.x + r2X;
			p2Y = bB.m_sweep.c.y + r2Y;
			
			
			
			this.m_u1.Set(p1X - s1X, p1Y - s1Y);
			
			this.m_u2.Set(p2X - s2X, p2Y - s2Y);
			
			length1 = this.m_u1.Length();
			length2 = this.m_u2.Length();
			
			if (length1 > b2Settings.b2_linearSlop)
			{
				
				this.m_u1.Multiply( 1.0 / length1 );
			}
			else
			{
				this.m_u1.SetZero();
			}
			
			if (length2 > b2Settings.b2_linearSlop)
			{
				
				this.m_u2.Multiply( 1.0 / length2 );
			}
			else
			{
				this.m_u2.SetZero();
			}
			
			C = this.m_constant - length1 - this.m_ratio * length2;
			linearError = b2Math.Max(linearError, -C);
			C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -this.m_pulleyMass * C;
			
			p1X = -impulse * this.m_u1.x;
			p1Y = -impulse * this.m_u1.y;
			p2X = -this.m_ratio * impulse * this.m_u2.x;
			p2Y = -this.m_ratio * impulse * this.m_u2.y;
			
			bA.m_sweep.c.x += bA.m_invMass * p1X;
			bA.m_sweep.c.y += bA.m_invMass * p1Y;
			bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
			bB.m_sweep.c.x += bB.m_invMass * p2X;
			bB.m_sweep.c.y += bB.m_invMass * p2Y;
			bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);
			
			bA.SynchronizeTransform();
			bB.SynchronizeTransform();
		}
		
		if (this.m_limitState1 == b2Joint.e_atUpperLimit)
		{
			
			tMat = bA.m_xf.R;
			r1X = this.m_localAnchor1.x - bA.m_sweep.localCenter.x;
			r1Y = this.m_localAnchor1.y - bA.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r1X + tMat.col2.x * r1Y);
			r1Y = (tMat.col1.y * r1X + tMat.col2.y * r1Y);
			r1X = tX;
			
			p1X = bA.m_sweep.c.x + r1X;
			p1Y = bA.m_sweep.c.y + r1Y;
			
			
			this.m_u1.Set(p1X - s1X, p1Y - s1Y);
			
			length1 = this.m_u1.Length();
			
			if (length1 > b2Settings.b2_linearSlop)
			{
				
				this.m_u1.x *= 1.0 / length1;
				this.m_u1.y *= 1.0 / length1;
			}
			else
			{
				this.m_u1.SetZero();
			}
			
			C = this.m_maxLength1 - length1;
			linearError = b2Math.Max(linearError, -C);
			C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -this.m_limitMass1 * C;
			
			
			p1X = -impulse * this.m_u1.x;
			p1Y = -impulse * this.m_u1.y;
			
			bA.m_sweep.c.x += bA.m_invMass * p1X;
			bA.m_sweep.c.y += bA.m_invMass * p1Y;
			
			bA.m_sweep.a += bA.m_invI * (r1X * p1Y - r1Y * p1X);
			
			bA.SynchronizeTransform();
		}
		
		if (this.m_limitState2 == b2Joint.e_atUpperLimit)
		{
			
			tMat = bB.m_xf.R;
			r2X = this.m_localAnchor2.x - bB.m_sweep.localCenter.x;
			r2Y = this.m_localAnchor2.y - bB.m_sweep.localCenter.y;
			tX = (tMat.col1.x * r2X + tMat.col2.x * r2Y);
			r2Y = (tMat.col1.y * r2X + tMat.col2.y * r2Y);
			r2X = tX;
			
			p2X = bB.m_sweep.c.x + r2X;
			p2Y = bB.m_sweep.c.y + r2Y;
			
			
			this.m_u2.Set(p2X - s2X, p2Y - s2Y);
			
			length2 = this.m_u2.Length();
			
			if (length2 > b2Settings.b2_linearSlop)
			{
				
				this.m_u2.x *= 1.0 / length2;
				this.m_u2.y *= 1.0 / length2;
			}
			else
			{
				this.m_u2.SetZero();
			}
			
			C = this.m_maxLength2 - length2;
			linearError = b2Math.Max(linearError, -C);
			C = b2Math.Clamp(C + b2Settings.b2_linearSlop, -b2Settings.b2_maxLinearCorrection, 0.0);
			impulse = -this.m_limitMass2 * C;
			
			
			p2X = -impulse * this.m_u2.x;
			p2Y = -impulse * this.m_u2.y;
			
			
			bB.m_sweep.c.x += bB.m_invMass * p2X;
			bB.m_sweep.c.y += bB.m_invMass * p2Y;
			
			bB.m_sweep.a += bB.m_invI * (r2X * p2Y - r2Y * p2X);
			
			bB.SynchronizeTransform();
		}
		
		return linearError < b2Settings.b2_linearSlop;
	}
b2PulleyJoint.prototype.GetAnchorA = function () {
		return this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
	}
b2PulleyJoint.prototype.GetAnchorB = function () {
		return this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
	}
b2PulleyJoint.prototype.GetReactionForce = function (inv_dt) {
		
		
		return new b2Vec2(inv_dt * this.m_impulse * this.m_u2.x, inv_dt * this.m_impulse * this.m_u2.y);
	}
b2PulleyJoint.prototype.GetReactionTorque = function (inv_dt) {
		
		return 0.0;
	}
b2PulleyJoint.prototype.GetGroundAnchorA = function () {
		
		var a = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor1);
		return a;
	}
b2PulleyJoint.prototype.GetGroundAnchorB = function () {
		
		var a = this.m_ground.m_xf.position.Copy();
		a.Add(this.m_groundAnchor2);
		return a;
	}
b2PulleyJoint.prototype.GetLength1 = function () {
		var p = this.m_bodyA.GetWorldPoint(this.m_localAnchor1);
		
		var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor1.x;
		var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor1.y;
		
		var dX = p.x - sX;
		var dY = p.y - sY;
		
		return Math.sqrt(dX*dX + dY*dY);
	}
b2PulleyJoint.prototype.GetLength2 = function () {
		var p = this.m_bodyB.GetWorldPoint(this.m_localAnchor2);
		
		var sX = this.m_ground.m_xf.position.x + this.m_groundAnchor2.x;
		var sY = this.m_ground.m_xf.position.y + this.m_groundAnchor2.y;
		
		var dX = p.x - sX;
		var dY = p.y - sY;
		
		return Math.sqrt(dX*dX + dY*dY);
	}
b2PulleyJoint.prototype.GetRatio = function () {
		return this.m_ratio;
	}
// attributes
b2PulleyJoint.prototype.m_ground =  null;
b2PulleyJoint.prototype.m_groundAnchor1 =  new b2Vec2();
b2PulleyJoint.prototype.m_groundAnchor2 =  new b2Vec2();
b2PulleyJoint.prototype.m_localAnchor1 =  new b2Vec2();
b2PulleyJoint.prototype.m_localAnchor2 =  new b2Vec2();
b2PulleyJoint.prototype.m_u1 =  new b2Vec2();
b2PulleyJoint.prototype.m_u2 =  new b2Vec2();
b2PulleyJoint.prototype.m_constant =  null;
b2PulleyJoint.prototype.m_ratio =  null;
b2PulleyJoint.prototype.m_maxLength1 =  null;
b2PulleyJoint.prototype.m_maxLength2 =  null;
b2PulleyJoint.prototype.m_pulleyMass =  null;
b2PulleyJoint.prototype.m_limitMass1 =  null;
b2PulleyJoint.prototype.m_limitMass2 =  null;
b2PulleyJoint.prototype.m_impulse =  null;
b2PulleyJoint.prototype.m_limitImpulse1 =  null;
b2PulleyJoint.prototype.m_limitImpulse2 =  null;
b2PulleyJoint.prototype.m_state =  0;
b2PulleyJoint.prototype.m_limitState1 =  0;
b2PulleyJoint.prototype.m_limitState2 =  0;// aliases  
b2PulleyJoint.prototype.initVelocityConstraints = b2PulleyJoint.prototype.InitVelocityConstraints; 
b2PulleyJoint.prototype.solveVelocityConstraints = b2PulleyJoint.prototype.SolveVelocityConstraints; 
b2PulleyJoint.prototype.solvePositionConstraints = b2PulleyJoint.prototype.SolvePositionConstraints; 
b2PulleyJoint.prototype.getAnchorA = b2PulleyJoint.prototype.GetAnchorA; 
b2PulleyJoint.prototype.getAnchorB = b2PulleyJoint.prototype.GetAnchorB; 
b2PulleyJoint.prototype.getReactionForce = b2PulleyJoint.prototype.GetReactionForce; 
b2PulleyJoint.prototype.getReactionTorque = b2PulleyJoint.prototype.GetReactionTorque; 
b2PulleyJoint.prototype.getGroundAnchorA = b2PulleyJoint.prototype.GetGroundAnchorA; 
b2PulleyJoint.prototype.getGroundAnchorB = b2PulleyJoint.prototype.GetGroundAnchorB; 
b2PulleyJoint.prototype.getLength1 = b2PulleyJoint.prototype.GetLength1; 
b2PulleyJoint.prototype.getLength2 = b2PulleyJoint.prototype.GetLength2; 
b2PulleyJoint.prototype.getRatio = b2PulleyJoint.prototype.GetRatio; 
// exports  
module.exports =  b2PulleyJoint;
