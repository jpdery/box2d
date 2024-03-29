// requires 
var b2Vec2 = require('../../common/math/vec2.js'); 
var b2Settings = require('../../common/settings.js'); 
var b2Math = require('../../common/math/math.js'); 
var b2EdgeShape = function() {
b2Shape.prototype.__varz.call(this)
this.__varz();
this.__constructor.apply(this, arguments);
}
extend(b2EdgeShape.prototype, b2Shape.prototype)
b2EdgeShape.prototype._super = b2Shape.prototype;
b2EdgeShape.prototype.__constructor = function (v1, v2) {
		this._super.__constructor.apply(this, []);
		this.m_type = b2Shape.e_edgeShape;
		
		this.m_prevEdge = null;
		this.m_nextEdge = null;
		
		this.m_v1 = v1;
		this.m_v2 = v2;
		
		this.m_direction.Set(this.m_v2.x - this.m_v1.x, this.m_v2.y - this.m_v1.y);
		this.m_length = this.m_direction.Normalize();
		this.m_normal.Set(this.m_direction.y, -this.m_direction.x);
		
		this.m_coreV1.Set(-b2Settings.b2_toiSlop * (this.m_normal.x - this.m_direction.x) + this.m_v1.x,
		 -b2Settings.b2_toiSlop * (this.m_normal.y - this.m_direction.y) + this.m_v1.y)
		this.m_coreV2.Set(-b2Settings.b2_toiSlop * (this.m_normal.x + this.m_direction.x) + this.m_v2.x,
		 -b2Settings.b2_toiSlop * (this.m_normal.y + this.m_direction.y) + this.m_v2.y)
		
		this.m_cornerDir1 = this.m_normal;
		this.m_cornerDir2.Set(-this.m_normal.x, -this.m_normal.y);
	}
b2EdgeShape.prototype.__varz = function(){
this.s_supportVec =  new b2Vec2();
this.m_v1 =  new b2Vec2();
this.m_v2 =  new b2Vec2();
this.m_coreV1 =  new b2Vec2();
this.m_coreV2 =  new b2Vec2();
this.m_normal =  new b2Vec2();
this.m_direction =  new b2Vec2();
this.m_cornerDir1 =  new b2Vec2();
this.m_cornerDir2 =  new b2Vec2();
}
// static methods
// static attributes
// methods
b2EdgeShape.prototype.SetPrevEdge = function (edge, core, cornerDir, convex) {
		this.m_prevEdge = edge;
		this.m_coreV1 = core;
		this.m_cornerDir1 = cornerDir;
		this.m_cornerConvex1 = convex;
	}
b2EdgeShape.prototype.SetNextEdge = function (edge, core, cornerDir, convex) {
		this.m_nextEdge = edge;
		this.m_coreV2 = core;
		this.m_cornerDir2 = cornerDir;
		this.m_cornerConvex2 = convex;
	}
b2EdgeShape.prototype.TestPoint = function (transform, p) {
		return false;
	}
b2EdgeShape.prototype.RayCast = function (output, input, transform) {
		var tMat;
		var rX = input.p2.x - input.p1.x;
		var rY = input.p2.y - input.p1.y;
		
		
		tMat = transform.R;
		var v1X = transform.position.x + (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
		var v1Y = transform.position.y + (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);
		
		
		var nX = transform.position.y + (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y) - v1Y;
		var nY = -(transform.position.x + (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y) - v1X);
		
		var k_slop = 100.0 * Number.MIN_VALUE;
		var denom = -(rX * nX + rY * nY);
	
		
		if (denom > k_slop)
		{
			
			var bX = input.p1.x - v1X;
			var bY = input.p1.y - v1Y;
			var a = (bX * nX + bY * nY);
	
			if (0.0 <= a && a <= input.maxFraction * denom)
			{
				var mu2 = -rX * bY + rY * bX;
	
				
				if (-k_slop * denom <= mu2 && mu2 <= denom * (1.0 + k_slop))
				{
					a /= denom;
					output.fraction = a;
					var nLen = Math.sqrt(nX * nX + nY * nY);
					output.normal.x = nX / nLen;
					output.normal.y = nY / nLen;
					return true;
				}
			}
		}
		
		return false;
	}
b2EdgeShape.prototype.ComputeAABB = function (aabb, transform) {
		var tMat = transform.R;
		
		var v1X = transform.position.x + (tMat.col1.x * this.m_v1.x + tMat.col2.x * this.m_v1.y);
		var v1Y = transform.position.y + (tMat.col1.y * this.m_v1.x + tMat.col2.y * this.m_v1.y);
		
		var v2X = transform.position.x + (tMat.col1.x * this.m_v2.x + tMat.col2.x * this.m_v2.y);
		var v2Y = transform.position.y + (tMat.col1.y * this.m_v2.x + tMat.col2.y * this.m_v2.y);
		if (v1X < v2X) {
			aabb.lowerBound.x = v1X;
			aabb.upperBound.x = v2X;
		} else {
			aabb.lowerBound.x = v2X;
			aabb.upperBound.x = v1X;
		}
		if (v1Y < v2Y) {
			aabb.lowerBound.y = v1Y;
			aabb.upperBound.y = v2Y;
		} else {
			aabb.lowerBound.y = v2Y;
			aabb.upperBound.y = v1Y;
		}
	}
b2EdgeShape.prototype.ComputeMass = function (massData, density) {
		massData.mass = 0;
		massData.center.SetV(this.m_v1);
		massData.I = 0;
	}
b2EdgeShape.prototype.ComputeSubmergedArea = function (
			normal,
			offset,
			xf,
			c) {
		
		
		
		var v0 = new b2Vec2(normal.x * offset, normal.y * offset);
		
		var v1 = b2Math.MulX(xf, this.m_v1);
		var v2 = b2Math.MulX(xf, this.m_v2);
		
		var d1 = b2Math.Dot(normal, v1) - offset;
		var d2 = b2Math.Dot(normal, v2) - offset;
		if (d1 > 0)
		{
			if (d2 > 0)
			{
				return 0;
			}
			else
			{
				
				v1.x = -d2 / (d1 - d2) * v1.x + d1 / (d1 - d2) * v2.x;
				v1.y = -d2 / (d1 - d2) * v1.y + d1 / (d1 - d2) * v2.y;
			}
		}
		else
		{
			if (d2 > 0)
			{
				
				v2.x = -d2 / (d1 - d2) * v1.x + d1 / (d1 - d2) * v2.x;
				v2.y = -d2 / (d1 - d2) * v1.y + d1 / (d1 - d2) * v2.y;
			}
			else
			{
				
			}
		}
		
		
		c.x = (v0.x + v1.x + v2.x) / 3;
		c.y = (v0.y + v1.y + v2.y) / 3;
		
		
		
		
		return 0.5 * ( (v1.x - v0.x) * (v2.y - v0.y) - (v1.y - v0.y) * (v2.x - v0.x) );
	}
b2EdgeShape.prototype.GetLength = function () {
		return this.m_length;
	}
b2EdgeShape.prototype.GetVertex1 = function () {
		return this.m_v1;
	}
b2EdgeShape.prototype.GetVertex2 = function () {
		return this.m_v2;
	}
b2EdgeShape.prototype.GetCoreVertex1 = function () {
		return this.m_coreV1;
	}
b2EdgeShape.prototype.GetCoreVertex2 = function () {
		return this.m_coreV2;
	}
b2EdgeShape.prototype.GetNormalVector = function () {
		return this.m_normal;
	}
b2EdgeShape.prototype.GetDirectionVector = function () {
		return this.m_direction;
	}
b2EdgeShape.prototype.GetCorner1Vector = function () {
		return this.m_cornerDir1;
	}
b2EdgeShape.prototype.GetCorner2Vector = function () {
		return this.m_cornerDir2;
	}
b2EdgeShape.prototype.Corner1IsConvex = function () {
		return this.m_cornerConvex1;
	}
b2EdgeShape.prototype.Corner2IsConvex = function () {
		return this.m_cornerConvex2;
	}
b2EdgeShape.prototype.GetFirstVertex = function (xf) {
		
		var tMat = xf.R;
		return new b2Vec2(xf.position.x + (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y),
		 xf.position.y + (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y));
	}
b2EdgeShape.prototype.GetNextEdge = function () {
		return this.m_nextEdge;
	}
b2EdgeShape.prototype.GetPrevEdge = function () {
		return this.m_prevEdge;
	}
b2EdgeShape.prototype.Support = function (xf, dX, dY) {
		var tMat = xf.R;
		
		var v1X = xf.position.x + (tMat.col1.x * this.m_coreV1.x + tMat.col2.x * this.m_coreV1.y);
		var v1Y = xf.position.y + (tMat.col1.y * this.m_coreV1.x + tMat.col2.y * this.m_coreV1.y);
		
		
		var v2X = xf.position.x + (tMat.col1.x * this.m_coreV2.x + tMat.col2.x * this.m_coreV2.y);
		var v2Y = xf.position.y + (tMat.col1.y * this.m_coreV2.x + tMat.col2.y * this.m_coreV2.y);
		
		if ((v1X * dX + v1Y * dY) > (v2X * dX + v2Y * dY)) {
			this.s_supportVec.x = v1X;
			this.s_supportVec.y = v1Y;
		} else {
			this.s_supportVec.x = v2X;
			this.s_supportVec.y = v2Y;
		}
		return this.s_supportVec;
	}
// attributes
b2EdgeShape.prototype.s_supportVec =  new b2Vec2();
b2EdgeShape.prototype.m_v1 =  new b2Vec2();
b2EdgeShape.prototype.m_v2 =  new b2Vec2();
b2EdgeShape.prototype.m_coreV1 =  new b2Vec2();
b2EdgeShape.prototype.m_coreV2 =  new b2Vec2();
b2EdgeShape.prototype.m_length =  null;
b2EdgeShape.prototype.m_normal =  new b2Vec2();
b2EdgeShape.prototype.m_direction =  new b2Vec2();
b2EdgeShape.prototype.m_cornerDir1 =  new b2Vec2();
b2EdgeShape.prototype.m_cornerDir2 =  new b2Vec2();
b2EdgeShape.prototype.m_cornerConvex1 =  null;
b2EdgeShape.prototype.m_cornerConvex2 =  null;
b2EdgeShape.prototype.m_nextEdge =  null;
b2EdgeShape.prototype.m_prevEdge =  null;// aliases  
b2EdgeShape.prototype.setPrevEdge = b2EdgeShape.prototype.SetPrevEdge; 
b2EdgeShape.prototype.setNextEdge = b2EdgeShape.prototype.SetNextEdge; 
b2EdgeShape.prototype.testPoint = b2EdgeShape.prototype.TestPoint; 
b2EdgeShape.prototype.rayCast = b2EdgeShape.prototype.RayCast; 
b2EdgeShape.prototype.computeAABB = b2EdgeShape.prototype.ComputeAABB; 
b2EdgeShape.prototype.computeMass = b2EdgeShape.prototype.ComputeMass; 
b2EdgeShape.prototype.computeSubmergedArea = b2EdgeShape.prototype.ComputeSubmergedArea; 
b2EdgeShape.prototype.getLength = b2EdgeShape.prototype.GetLength; 
b2EdgeShape.prototype.getVertex1 = b2EdgeShape.prototype.GetVertex1; 
b2EdgeShape.prototype.getVertex2 = b2EdgeShape.prototype.GetVertex2; 
b2EdgeShape.prototype.getCoreVertex1 = b2EdgeShape.prototype.GetCoreVertex1; 
b2EdgeShape.prototype.getCoreVertex2 = b2EdgeShape.prototype.GetCoreVertex2; 
b2EdgeShape.prototype.getNormalVector = b2EdgeShape.prototype.GetNormalVector; 
b2EdgeShape.prototype.getDirectionVector = b2EdgeShape.prototype.GetDirectionVector; 
b2EdgeShape.prototype.getCorner1Vector = b2EdgeShape.prototype.GetCorner1Vector; 
b2EdgeShape.prototype.getCorner2Vector = b2EdgeShape.prototype.GetCorner2Vector; 
b2EdgeShape.prototype.corner1IsConvex = b2EdgeShape.prototype.Corner1IsConvex; 
b2EdgeShape.prototype.corner2IsConvex = b2EdgeShape.prototype.Corner2IsConvex; 
b2EdgeShape.prototype.getFirstVertex = b2EdgeShape.prototype.GetFirstVertex; 
b2EdgeShape.prototype.getNextEdge = b2EdgeShape.prototype.GetNextEdge; 
b2EdgeShape.prototype.getPrevEdge = b2EdgeShape.prototype.GetPrevEdge; 
b2EdgeShape.prototype.support = b2EdgeShape.prototype.Support; 
// exports  
module.exports =  b2EdgeShape;
