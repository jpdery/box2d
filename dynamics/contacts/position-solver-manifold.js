// requires 
var b2Vec2 = require('../../common/math/vec2.js'); 
var b2Settings = require('../../common/settings.js'); 
var b2PositionSolverManifold = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2PositionSolverManifold.prototype.__constructor = function () {
		this.m_normal = new b2Vec2();
		this.m_separations = new Array(b2Settings.b2_maxManifoldPoints);
		this.m_points = new Array(b2Settings.b2_maxManifoldPoints);
		for (var i = 0; i < b2Settings.b2_maxManifoldPoints; i++)
		{
			this.m_points[i] = new b2Vec2();
		}
	}
b2PositionSolverManifold.prototype.__varz = function(){
}
// static methods
// static attributes
b2PositionSolverManifold.circlePointA =  new b2Vec2();
b2PositionSolverManifold.circlePointB =  new b2Vec2();
// methods
b2PositionSolverManifold.prototype.Initialize = function (cc) {
		b2Settings.b2Assert(cc.pointCount > 0);
		
		var i = 0;
		var clipPointX;
		var clipPointY;
		var tMat;
		var tVec;
		var planePointX;
		var planePointY;
		
		switch(cc.type)
		{
			case b2Manifold.e_circles:
			{
				
				tMat = cc.bodyA.m_xf.R;
				tVec = cc.localPoint;
				var pointAX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				var pointAY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				
				tMat = cc.bodyB.m_xf.R;
				tVec = cc.points[0].localPoint;
				var pointBX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				var pointBY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				var dX = pointBX - pointAX;
				var dY = pointBY - pointAY;
				var d2 = dX * dX + dY * dY;
				if (d2 > Number.MIN_VALUE*Number.MIN_VALUE)
				{
					var d = Math.sqrt(d2);
					this.m_normal.x = dX/d;
					this.m_normal.y = dY/d;
				}
				else
				{
					this.m_normal.x = 1.0;
					this.m_normal.y = 0.0;
				}
				this.m_points[0].x = 0.5 * (pointAX + pointBX);
				this.m_points[0].y = 0.5 * (pointAY + pointBY);
				this.m_separations[0] = dX * this.m_normal.x + dY * this.m_normal.y - cc.radius;
			}
			break;
			case b2Manifold.e_faceA:
			{
				
				tMat = cc.bodyA.m_xf.R;
				tVec = cc.localPlaneNormal;
				this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
				
				tMat = cc.bodyA.m_xf.R;
				tVec = cc.localPoint;
				planePointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				planePointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				
				tMat = cc.bodyB.m_xf.R;
				for (i = 0; i < cc.pointCount;++i)
				{
					
					tVec = cc.points[i].localPoint;
					clipPointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
					clipPointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
					this.m_separations[i] = (clipPointX - planePointX) * this.m_normal.x + (clipPointY - planePointY) * this.m_normal.y - cc.radius;
					this.m_points[i].x = clipPointX;
					this.m_points[i].y = clipPointY;
				}
			}
			break;
			case b2Manifold.e_faceB:
			{
				
				tMat = cc.bodyB.m_xf.R;
				tVec = cc.localPlaneNormal;
				this.m_normal.x = tMat.col1.x * tVec.x + tMat.col2.x * tVec.y;
				this.m_normal.y = tMat.col1.y * tVec.x + tMat.col2.y * tVec.y;
				
				tMat = cc.bodyB.m_xf.R;
				tVec = cc.localPoint;
				planePointX = cc.bodyB.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
				planePointY = cc.bodyB.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
				
				tMat = cc.bodyA.m_xf.R;
				for (i = 0; i < cc.pointCount;++i)
				{
					
					tVec = cc.points[i].localPoint;
					clipPointX = cc.bodyA.m_xf.position.x + (tMat.col1.x * tVec.x + tMat.col2.x * tVec.y);
					clipPointY = cc.bodyA.m_xf.position.y + (tMat.col1.y * tVec.x + tMat.col2.y * tVec.y);
					this.m_separations[i] = (clipPointX - planePointX) * this.m_normal.x + (clipPointY - planePointY) * this.m_normal.y - cc.radius;
					this.m_points[i].Set(clipPointX, clipPointY);
				}
				
				
				this.m_normal.x *= -1;
				this.m_normal.y *= -1;
			}
			break;
		}
	}
// attributes
b2PositionSolverManifold.prototype.m_normal =  null;
b2PositionSolverManifold.prototype.m_points =  null;
b2PositionSolverManifold.prototype.m_separations =  null;// aliases  
b2PositionSolverManifold.prototype.initialize = b2PositionSolverManifold.prototype.Initialize; 
// exports  
module.exports =  b2PositionSolverManifold;
