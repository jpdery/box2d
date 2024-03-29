// requires 
var b2JointEdge = require('./joint-edge.js'); 
var b2Vec2 = require('../../common/math/vec2.js'); 
var b2DistanceJoint = require('./distance-joint.js'); 
var b2MouseJoint = require('./mouse-joint.js'); 
var b2PrismaticJoint = require('./prismatic-joint.js'); 
var b2RevoluteJoint = require('./revolute-joint.js'); 
var b2PulleyJoint = require('./pulley-joint.js'); 
var b2GearJoint = require('./gear-joint.js'); 
var b2LineJoint = require('./line-joint.js'); 
var b2WeldJoint = require('./weld-joint.js'); 
var b2FrictionJoint = require('./friction-joint.js'); 
var b2Settings = require('../../common/settings.js'); 
var b2Joint = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Joint.prototype.__constructor = function (def) {
		b2Settings.b2Assert(def.bodyA != def.bodyB);
		this.m_type = def.type;
		this.m_prev = null;
		this.m_next = null;
		this.m_bodyA = def.bodyA;
		this.m_bodyB = def.bodyB;
		this.m_collideConnected = def.collideConnected;
		this.m_islandFlag = false;
		this.m_userData = def.userData;
	}
b2Joint.prototype.__varz = function(){
this.m_edgeA =  new b2JointEdge();
this.m_edgeB =  new b2JointEdge();
this.m_localCenterA =  new b2Vec2();
this.m_localCenterB =  new b2Vec2();
}
// static methods
b2Joint.Create = function (def, allocator) {
		var joint = null;
		
		switch (def.type)
		{
		case b2Joint.e_distanceJoint:
			{
				
				joint = new b2DistanceJoint(def);
			}
			break;
		
		case b2Joint.e_mouseJoint:
			{
				
				joint = new b2MouseJoint(def);
			}
			break;
		
		case b2Joint.e_prismaticJoint:
			{
				
				joint = new b2PrismaticJoint(def);
			}
			break;
		
		case b2Joint.e_revoluteJoint:
			{
				
				joint = new b2RevoluteJoint(def);
			}
			break;
		
		case b2Joint.e_pulleyJoint:
			{
				
				joint = new b2PulleyJoint(def);
			}
			break;
		
		case b2Joint.e_gearJoint:
			{
				
				joint = new b2GearJoint(def);
			}
			break;
		
		case b2Joint.e_lineJoint:
			{
				
				joint = new b2LineJoint(def);
			}
			break;
			
		case b2Joint.e_weldJoint:
			{
				
				joint = new b2WeldJoint(def);
			}
			break;
			
		case b2Joint.e_frictionJoint:
			{
				
				joint = new b2FrictionJoint(def);
			}
			break;
			
		default:
			
			break;
		}
		
		return joint;
	}
b2Joint.Destroy = function (joint, allocator) {
		
	}
// static attributes
b2Joint.e_unknownJoint =  0;
b2Joint.e_revoluteJoint =  1;
b2Joint.e_prismaticJoint =  2;
b2Joint.e_distanceJoint =  3;
b2Joint.e_pulleyJoint =  4;
b2Joint.e_mouseJoint =  5;
b2Joint.e_gearJoint =  6;
b2Joint.e_lineJoint =  7;
b2Joint.e_weldJoint =  8;
b2Joint.e_frictionJoint =  9;
b2Joint.e_inactiveLimit =  0;
b2Joint.e_atLowerLimit =  1;
b2Joint.e_atUpperLimit =  2;
b2Joint.e_equalLimits =  3;
// methods
b2Joint.prototype.InitVelocityConstraints = function (step) {}
b2Joint.prototype.SolveVelocityConstraints = function (step) { }
b2Joint.prototype.FinalizeVelocityConstraints = function () {}
b2Joint.prototype.SolvePositionConstraints = function (baumgarte) { return false }
b2Joint.prototype.GetType = function () {
		return this.m_type;
	}
b2Joint.prototype.GetAnchorA = function () {return null}
b2Joint.prototype.GetAnchorB = function () {return null}
b2Joint.prototype.GetReactionForce = function (inv_dt) {return null}
b2Joint.prototype.GetReactionTorque = function (inv_dt) {return 0.0}
b2Joint.prototype.GetBodyA = function () {
		return this.m_bodyA;
	}
b2Joint.prototype.GetBodyB = function () {
		return this.m_bodyB;
	}
b2Joint.prototype.GetNext = function () {
		return this.m_next;
	}
b2Joint.prototype.GetUserData = function () {
		return this.m_userData;
	}
b2Joint.prototype.SetUserData = function (data) {
		this.m_userData = data;
	}
b2Joint.prototype.IsActive = function () {
		return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
	}
// attributes
b2Joint.prototype.m_type =  0;
b2Joint.prototype.m_prev =  null;
b2Joint.prototype.m_next =  null;
b2Joint.prototype.m_edgeA =  new b2JointEdge();
b2Joint.prototype.m_edgeB =  new b2JointEdge();
b2Joint.prototype.m_bodyA =  null;
b2Joint.prototype.m_bodyB =  null;
b2Joint.prototype.m_islandFlag =  null;
b2Joint.prototype.m_collideConnected =  null;
b2Joint.prototype.m_userData =  null;
b2Joint.prototype.m_localCenterA =  new b2Vec2();
b2Joint.prototype.m_localCenterB =  new b2Vec2();
b2Joint.prototype.m_invMassA =  null;
b2Joint.prototype.m_invMassB =  null;
b2Joint.prototype.m_invIA =  null;
b2Joint.prototype.m_invIB =  null;// aliases  
b2Joint.prototype.initVelocityConstraints = b2Joint.prototype.InitVelocityConstraints; 
b2Joint.prototype.solveVelocityConstraints = b2Joint.prototype.SolveVelocityConstraints; 
b2Joint.prototype.finalizeVelocityConstraints = b2Joint.prototype.FinalizeVelocityConstraints; 
b2Joint.prototype.solvePositionConstraints = b2Joint.prototype.SolvePositionConstraints; 
b2Joint.prototype.getType = b2Joint.prototype.GetType; 
b2Joint.prototype.getAnchorA = b2Joint.prototype.GetAnchorA; 
b2Joint.prototype.getAnchorB = b2Joint.prototype.GetAnchorB; 
b2Joint.prototype.getReactionForce = b2Joint.prototype.GetReactionForce; 
b2Joint.prototype.getReactionTorque = b2Joint.prototype.GetReactionTorque; 
b2Joint.prototype.getBodyA = b2Joint.prototype.GetBodyA; 
b2Joint.prototype.getBodyB = b2Joint.prototype.GetBodyB; 
b2Joint.prototype.getNext = b2Joint.prototype.GetNext; 
b2Joint.prototype.getUserData = b2Joint.prototype.GetUserData; 
b2Joint.prototype.setUserData = b2Joint.prototype.SetUserData; 
b2Joint.prototype.isActive = b2Joint.prototype.IsActive; 
// exports  
module.exports =  b2Joint;
