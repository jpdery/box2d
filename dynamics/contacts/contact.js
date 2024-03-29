// requires 
var b2ContactEdge = require('./contact-edge.js'); 
var b2Manifold = require('../../collision/manifold.js'); 
var b2TOIInput = require('../../collision/toiinput.js'); 
var b2Settings = require('../../common/settings.js'); 
var b2Contact = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Contact.prototype.__constructor = function () {
		
	}
b2Contact.prototype.__varz = function(){
this.m_nodeA =  new b2ContactEdge();
this.m_nodeB =  new b2ContactEdge();
this.m_manifold =  new b2Manifold();
this.m_oldManifold =  new b2Manifold();
}
// static methods
// static attributes
b2Contact.s_input =  new b2TOIInput();
b2Contact.e_sensorFlag =  0x0001;
b2Contact.e_continuousFlag =  0x0002;
b2Contact.e_islandFlag =  0x0004;
b2Contact.e_toiFlag =  0x0008;
b2Contact.e_touchingFlag =  0x0010;
b2Contact.e_enabledFlag =  0x0020;
b2Contact.e_filterFlag =  0x0040;
// methods
b2Contact.prototype.Reset = function (fixtureA , fixtureB ) {
		this.m_flags = b2Contact.e_enabledFlag;
		
		if (!fixtureA || !fixtureB){
			this.m_fixtureA = null;
			this.m_fixtureB = null;
			return;
		}
		
		if (fixtureA.IsSensor() || fixtureB.IsSensor())
		{
			this.m_flags |= b2Contact.e_sensorFlag;
		}
		
		var bodyA = fixtureA.GetBody();
		var bodyB = fixtureB.GetBody();
		
		if (bodyA.GetType() != b2Body.b2_dynamicBody || bodyA.IsBullet() || bodyB.GetType() != b2Body.b2_dynamicBody || bodyB.IsBullet())
		{
			this.m_flags |= b2Contact.e_continuousFlag;
		}
		
		this.m_fixtureA = fixtureA;
		this.m_fixtureB = fixtureB;
		
		this.m_manifold.m_pointCount = 0;
		
		this.m_prev = null;
		this.m_next = null;
		
		this.m_nodeA.contact = null;
		this.m_nodeA.prev = null;
		this.m_nodeA.next = null;
		this.m_nodeA.other = null;
		
		this.m_nodeB.contact = null;
		this.m_nodeB.prev = null;
		this.m_nodeB.next = null;
		this.m_nodeB.other = null;
	}
b2Contact.prototype.Update = function (listener) {
		
		var tManifold = this.m_oldManifold;
		this.m_oldManifold = this.m_manifold;
		this.m_manifold = tManifold;
		
		
		this.m_flags |= b2Contact.e_enabledFlag;
		
		var touching = false;
		var wasTouching = (this.m_flags & b2Contact.e_touchingFlag) == b2Contact.e_touchingFlag;
		
		var bodyA = this.m_fixtureA.m_body;
		var bodyB = this.m_fixtureB.m_body;
		
		var aabbOverlap = this.m_fixtureA.m_aabb.TestOverlap(this.m_fixtureB.m_aabb);
		
		
		if (this.m_flags & b2Contact.e_sensorFlag)
		{
			if (aabbOverlap)
			{
				var shapeA = this.m_fixtureA.GetShape();
				var shapeB = this.m_fixtureB.GetShape();
				var xfA = bodyA.GetTransform();
				var xfB = bodyB.GetTransform();
				touching = b2Shape.TestOverlap(shapeA, xfA, shapeB, xfB);
			}
			
			
			this.m_manifold.m_pointCount = 0;
		}
		else
		{
			
			if (bodyA.GetType() != b2Body.b2_dynamicBody || bodyA.IsBullet() || bodyB.GetType() != b2Body.b2_dynamicBody || bodyB.IsBullet())
			{
				this.m_flags |= b2Contact.e_continuousFlag;
			}
			else
			{
				this.m_flags &= ~b2Contact.e_continuousFlag;
			}
			
			if (aabbOverlap)
			{
				this.Evaluate();
				
				touching = this.m_manifold.m_pointCount > 0;
				
				
				
				for (var i = 0; i < this.m_manifold.m_pointCount; ++i)
				{
					var mp2 = this.m_manifold.m_points[i];
					mp2.m_normalImpulse = 0.0;
					mp2.m_tangentImpulse = 0.0;
					var id2 = mp2.m_id;

					for (var j = 0; j < this.m_oldManifold.m_pointCount; ++j)
					{
						var mp1 = this.m_oldManifold.m_points[j];

						if (mp1.m_id.key == id2.key)
						{
							mp2.m_normalImpulse = mp1.m_normalImpulse;
							mp2.m_tangentImpulse = mp1.m_tangentImpulse;
							break;
						}
					}
				}

			}
			else
			{
				this.m_manifold.m_pointCount = 0;
			}
			if (touching != wasTouching)
			{
				bodyA.SetAwake(true);
				bodyB.SetAwake(true);
			}
		}
				
		if (touching)
		{
			this.m_flags |= b2Contact.e_touchingFlag;
		}
		else
		{
			this.m_flags &= ~b2Contact.e_touchingFlag;
		}

		if (wasTouching == false && touching == true)
		{
			listener.BeginContact(this);
		}

		if (wasTouching == true && touching == false)
		{
			listener.EndContact(this);
		}

		if ((this.m_flags & b2Contact.e_sensorFlag) == 0)
		{
			listener.PreSolve(this, this.m_oldManifold);
		}
	}
b2Contact.prototype.Evaluate = function () {}
b2Contact.prototype.ComputeTOI = function (sweepA, sweepB) {
		b2Contact.s_input.proxyA.Set(this.m_fixtureA.GetShape());
		b2Contact.s_input.proxyB.Set(this.m_fixtureB.GetShape());
		b2Contact.s_input.sweepA = sweepA;
		b2Contact.s_input.sweepB = sweepB;
		b2Contact.s_input.tolerance = b2Settings.b2_linearSlop;
		return b2TimeOfImpact.TimeOfImpact(b2Contact.s_input);
	}
b2Contact.prototype.GetManifold = function () {
		return this.m_manifold;
	}
b2Contact.prototype.GetWorldManifold = function (worldManifold) {
		var bodyA = this.m_fixtureA.GetBody();
		var bodyB = this.m_fixtureB.GetBody();
		var shapeA = this.m_fixtureA.GetShape();
		var shapeB = this.m_fixtureB.GetShape();
		
		worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
	}
b2Contact.prototype.IsTouching = function () {
		return (this.m_flags & b2Contact.e_touchingFlag) == b2Contact.e_touchingFlag; 
	}
b2Contact.prototype.IsContinuous = function () {
		return (this.m_flags & b2Contact.e_continuousFlag) == b2Contact.e_continuousFlag; 
	}
b2Contact.prototype.SetSensor = function (sensor) {
		if (sensor)
		{
			this.m_flags |= b2Contact.e_sensorFlag;
		}
		else
		{
			this.m_flags &= ~b2Contact.e_sensorFlag;
		}
	}
b2Contact.prototype.IsSensor = function () {
		return (this.m_flags & b2Contact.e_sensorFlag) == b2Contact.e_sensorFlag;
	}
b2Contact.prototype.SetEnabled = function (flag) {
		if (flag)
		{
			this.m_flags |= b2Contact.e_enabledFlag;
		}
		else
		{
			this.m_flags &= ~b2Contact.e_enabledFlag;
		}
	}
b2Contact.prototype.IsEnabled = function () {
		return (this.m_flags & b2Contact.e_enabledFlag) == b2Contact.e_enabledFlag;
	}
b2Contact.prototype.GetNext = function () {
		return this.m_next;
	}
b2Contact.prototype.GetFixtureA = function () {
		return this.m_fixtureA;
	}
b2Contact.prototype.GetFixtureB = function () {
		return this.m_fixtureB;
	}
b2Contact.prototype.FlagForFiltering = function () {
		this.m_flags |= b2Contact.e_filterFlag;
	}
// attributes
b2Contact.prototype.m_flags =  0;
b2Contact.prototype.m_prev =  null;
b2Contact.prototype.m_next =  null;
b2Contact.prototype.m_nodeA =  new b2ContactEdge();
b2Contact.prototype.m_nodeB =  new b2ContactEdge();
b2Contact.prototype.m_fixtureA =  null;
b2Contact.prototype.m_fixtureB =  null;
b2Contact.prototype.m_manifold =  new b2Manifold();
b2Contact.prototype.m_oldManifold =  new b2Manifold();
b2Contact.prototype.m_toi =  null;// aliases  
b2Contact.prototype.reset = b2Contact.prototype.Reset; 
b2Contact.prototype.update = b2Contact.prototype.Update; 
b2Contact.prototype.evaluate = b2Contact.prototype.Evaluate; 
b2Contact.prototype.computeTOI = b2Contact.prototype.ComputeTOI; 
b2Contact.prototype.getManifold = b2Contact.prototype.GetManifold; 
b2Contact.prototype.getWorldManifold = b2Contact.prototype.GetWorldManifold; 
b2Contact.prototype.isTouching = b2Contact.prototype.IsTouching; 
b2Contact.prototype.isContinuous = b2Contact.prototype.IsContinuous; 
b2Contact.prototype.setSensor = b2Contact.prototype.SetSensor; 
b2Contact.prototype.isSensor = b2Contact.prototype.IsSensor; 
b2Contact.prototype.setEnabled = b2Contact.prototype.SetEnabled; 
b2Contact.prototype.isEnabled = b2Contact.prototype.IsEnabled; 
b2Contact.prototype.getNext = b2Contact.prototype.GetNext; 
b2Contact.prototype.getFixtureA = b2Contact.prototype.GetFixtureA; 
b2Contact.prototype.getFixtureB = b2Contact.prototype.GetFixtureB; 
b2Contact.prototype.flagForFiltering = b2Contact.prototype.FlagForFiltering; 
// exports  
module.exports =  b2Contact;
