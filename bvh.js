/**
 * Bounding Volume Hierarchy data structure
 */

/**
 * A 3D Vector class. Based on three.js Vector3
 */
BVHVector3 = function ( x, y, z ) {
    this.x = x || 0;
    this.y = y || 0;
    this.z = z || 0;

};

BVHVector3.prototype = {

    constructor: BVHVector3,

    copy: function ( v ) {

        this.x = v.x;
        this.y = v.y;
        this.z = v.z;

        return this;

    },

    set: function ( x, y, z ) {

        this.x = x;
        this.y = y;
        this.z = z;

        return this;
    },

    add: function ( v ) {

        this.x += v.x;
        this.y += v.y;
        this.z += v.z;

        return this;
    },

    multiplyScalar: function ( scalar ) {

        this.x *= scalar;
        this.y *= scalar;
        this.z *= scalar;

        return this;

    },

    subVectors: function ( a, b ) {

        this.x = a.x - b.x;
        this.y = a.y - b.y;
        this.z = a.z - b.z;

        return this;

    },

    dot: function ( v ) {

        return this.x * v.x + this.y * v.y + this.z * v.z;

    },

    cross: function ( v ) {
        var x = this.x, y = this.y, z = this.z;

        this.x = y * v.z - z * v.y;
        this.y = z * v.x - x * v.z;
        this.z = x * v.y - y * v.x;

        return this;
    },

    crossVectors: function ( a, b ) {

        var ax = a.x, ay = a.y, az = a.z;
        var bx = b.x, by = b.y, bz = b.z;

        this.x = ay * bz - az * by;
        this.y = az * bx - ax * bz;
        this.z = ax * by - ay * bx;

        return this;
    }
};

function BVH(trianglesArray, maxTrianglesPerNode) {
    this._trianglesArray = trianglesArray;
    this._maxTrianglesPerNode = maxTrianglesPerNode || 50;
    this._bboxArray = this.calcBoundingBoxes(trianglesArray);

    // clone a helper array
    this._bboxHelper = new Float32Array(this._bboxArray.length);
    this._bboxHelper.set(this._bboxArray);

    // create the root node, add all the triangles to it
    var triangleCount = trianglesArray.length / 9;
    var extents = this.calcExtents(0, triangleCount);
    this._rootNode = new BVHNode(extents[0], extents[1], 0, triangleCount, 0);

    this._nodesToSplit = [this._rootNode];

    while (this._nodesToSplit.length > 0) {
        var node = this._nodesToSplit.pop();
        this.splitNode(node);
    }
}

/**
 * returns a list of all the triangles in the BVH which interected a specific node.
 * We use the BVH node structure to first cull out nodes which do not intereset the ray.
 * For rays that did intersect, we test intersection of the ray with each triangle
 * @param rayOrigin origin position of the ray {x, y, z}
 * @param rayDirection direction of the ray {x, y, z}
 * @return {Array} an array of triangle indices (triangle positions in the input trianglesArray), which intersected the BVH
 */
BVH.prototype.intersectRay = function(rayOrigin, rayDirection, backfaceCulling) {
    var nodesToIntersect = [this._rootNode];
    var trianglesInIntersectingNodes = []; // a list of nodes that intersect the ray (according to their bounding box)
    var intersectingTriangles = [];

    // go over the BVH tree, and extract the list of triangles that lie in nodes that intersect the ray.
    // note: these triangles may not intersect the ray themselves
    while (nodesToIntersect.length > 0) {
        var node = nodesToIntersect.pop();

        if (BVH.intersectNodeBox(rayOrigin, rayDirection, node)) {
            if (node._node0) {
                nodesToIntersect.push(node._node0);
            }

            if (node._node1) {
                nodesToIntersect.push(node._node1);
            }

            for (var i = node._startIndex; i < node._endIndex; i++) {
                trianglesInIntersectingNodes.push(this._bboxArray[i*7]);
            }
        }
    }

    // go over the list of candidate triangles, and check each of them using ray triangle intersection
    var a = new BVHVector3();
    var b = new BVHVector3();
    var c = new BVHVector3();
    var rayOriginVec3 = new BVHVector3(rayOrigin.x, rayOrigin.y, rayOrigin.z);
    var rayDirectionVec3 = new BVHVector3(rayDirection.x, rayDirection.y, rayDirection.z);

    for (i = 0; i < trianglesInIntersectingNodes.length; i++) {
        var triIndex = trianglesInIntersectingNodes[i];

        a.set(this._trianglesArray[triIndex*9], this._trianglesArray[triIndex*9+1], this._trianglesArray[triIndex*9+2]);
        b.set(this._trianglesArray[triIndex*9+3], this._trianglesArray[triIndex*9+4], this._trianglesArray[triIndex*9+5]);
        c.set(this._trianglesArray[triIndex*9+6], this._trianglesArray[triIndex*9+7], this._trianglesArray[triIndex*9+8]);

        if (BVH.intersectRayTriangle(a, b, c, rayOriginVec3, rayDirectionVec3, backfaceCulling)) {
            intersectingTriangles.push(triIndex);
        }
    }

    return intersectingTriangles;
};

/**
 * Gets an array of triangle, and calculates the bounding box for each of them, and adds an index to the triangle's position in the array
 * Each bbox is saved as 7 values in a Float32Array: (position, minX, minY, minZ, maxX, maxY, maxZ)
 */
BVH.prototype.calcBoundingBoxes = function(trianglesArray) {
    var p0x, p0y, p0z;
    var p1x, p1y, p1z;
    var p2x, p2y, p2z;
    var minX, minY, minZ;
    var maxX, maxY, maxZ;

    var triangleCount = trianglesArray.length / 9;
    var bboxArray = new Float32Array(triangleCount * 7);

    for (var i = 0; i < triangleCount; i++) {
        p0x = trianglesArray[i*9];
        p0y = trianglesArray[i*9+1];
        p0z = trianglesArray[i*9+2];
        p1x = trianglesArray[i*9+3];
        p1y = trianglesArray[i*9+4];
        p1z = trianglesArray[i*9+5];
        p2x = trianglesArray[i*9+6];
        p2y = trianglesArray[i*9+7];
        p2z = trianglesArray[i*9+8];

        minX = Math.min(Math.min(p0x, p1x), p2x);
        minY = Math.min(Math.min(p0y, p1y), p2y);
        minZ = Math.min(Math.min(p0z, p1z), p2z);
        maxX = Math.max(Math.max(p0x, p1x), p2x);
        maxY = Math.max(Math.max(p0y, p1y), p2y);
        maxZ = Math.max(Math.max(p0z, p1z), p2z);

        BVH.setBox(bboxArray, i, i, minX, minY, minZ, maxX, maxY, maxZ);
    }

    return bboxArray;
};

/**
 * Calculates the extents (i.e the min and max coordinates) of a list of bounding boxes in the bboxArray
 * @param startIndex the index of the first triangle that we want to calc extents for
 * @param endIndex the index of the last triangle that we want to calc extents for
 */
BVH.prototype.calcExtents = function(startIndex, endIndex) {
    if (startIndex >= endIndex) {
        return [{'x': 0, 'y': 0, 'z': 0}, {'x': 0, 'y': 0, 'z': 0}]
    }

    var minX = Number.MAX_VALUE;
    var minY = Number.MAX_VALUE;
    var minZ = Number.MAX_VALUE;
    var maxX = -Number.MAX_VALUE;
    var maxY = -Number.MAX_VALUE;
    var maxZ = -Number.MAX_VALUE;

    for (var i = startIndex; i < endIndex; i++) {
        minX = Math.min(this._bboxArray[i*7+1], minX);
        minY = Math.min(this._bboxArray[i*7+2], minY);
        minZ = Math.min(this._bboxArray[i*7+3], minZ);
        maxX = Math.max(this._bboxArray[i*7+4], maxX);
        maxY = Math.max(this._bboxArray[i*7+5], maxY);
        maxZ = Math.max(this._bboxArray[i*7+6], maxZ);
    }

    return [{'x': minX, 'y': minY, 'z': minZ}, {'x': maxX, 'y': maxY, 'z': maxZ}];
};

BVH.prototype.splitNode = function(node) {
    if ((node.elementCount() <= this._maxTrianglesPerNode) || (node.elementCount() === 0)) {
        return;
    }

    // split rule is according to the node's level: first split by x axis, then by y axis, then by z axis and repeat..
    var startIndex = node._startIndex;
    var endIndex = node._endIndex;
    var objectCenterX, objectCenterY, objectCenterZ;
    var extentsCenterX, extentsCenterY, extentsCenterZ;
    var i;

    var node0XElements = [];
    var node1XElements = [];
    var node0YElements = [];
    var node1YElements = [];
    var node0ZElements = [];
    var node1ZElements = [];

    extentsCenterX = node.centerX();
    extentsCenterY = node.centerY();
    extentsCenterZ = node.centerZ();

    for (i = startIndex; i < endIndex; i++) {
        objectCenterX = (this._bboxArray[i * 7 + 1] + this._bboxArray[i * 7 + 4]) * 0.5; // center = (min + max) / 2
        objectCenterY = (this._bboxArray[i * 7 + 2] + this._bboxArray[i * 7 + 5]) * 0.5; // center = (min + max) / 2
        objectCenterZ = (this._bboxArray[i * 7 + 3] + this._bboxArray[i * 7 + 6]) * 0.5; // center = (min + max) / 2

        if (objectCenterX < extentsCenterX) {
            node0XElements.push(i);
        }
        else {
            node1XElements.push(i);
        }

        if (objectCenterY < extentsCenterY) {
            node0YElements.push(i);
        }
        else {
            node1YElements.push(i);
        }

        if (objectCenterZ < extentsCenterZ) {
            node0ZElements.push(i);
        }
        else {
            node1ZElements.push(i);
        }
    }

    // check if we couldn't split the node by any of the axes (x, y or z). halt here, dont try to split any more (cause it will always fail, and we'll enter an infinite loop
    var splitXFailed = (node0XElements.length === 0) || (node1XElements.length === 0);
    var splitYFailed = (node0YElements.length === 0) || (node1YElements.length === 0);
    var splitZFailed = (node0ZElements.length === 0) || (node1ZElements.length === 0);

    if (splitXFailed && splitYFailed && splitZFailed) {
        return;
    }

    // in case one of the splits failed, choose to split by the axis which resulted in the most balanced split of shapes between the two child nodes
    var defaultAxisFailed = false;
    if ((node._level % 3 === 0)) {
        if (splitXFailed) {
            defaultAxisFailed = true;
        }
        else {
            node0Elements = node0XElements;
            node1Elements = node1XElements;
        }
    }

    if ((node._level % 3 === 1)) {
        if (splitYFailed) {
            defaultAxisFailed = true;
        }
        else {
            node0Elements = node0YElements;
            node1Elements = node1YElements;
        }
    }

    if ((node._level % 3 === 2)) {
        if (splitZFailed) {
            defaultAxisFailed = true;
        }
        else {
            node0Elements = node0ZElements;
            node1Elements = node1ZElements;
        }
    }

    if (defaultAxisFailed) {
        var node0Elements = [];
        var node1Elements = [];
        var xImbalance = Math.abs(node0XElements.length - node1XElements.length);
        var yImbalance = Math.abs(node0YElements.length - node1YElements.length);
        var zImbalance = Math.abs(node0ZElements.length - node1ZElements.length);

        if (xImbalance < yImbalance) {
            if (xImbalance < zImbalance) { // split by x
                node0Elements = node0XElements;
                node1Elements = node1XElements;
            }
            else { // split by z
                node0Elements = node0ZElements;
                node1Elements = node1ZElements;
            }
        }
        else if (yImbalance < zImbalance) { // split by y
            node0Elements = node0YElements;
            node1Elements = node1YElements;
        }
        else { // split by z
            node0Elements = node0ZElements;
            node1Elements = node1ZElements;
        }
    }

    // sort the elements in range (startIndex, endIndex) according to which node they should be at
    var node0Start = startIndex;
    var node0End = node0Start + node0Elements.length;
    var node1Start = node0End;
    var node1End = endIndex;
    var currElement;

    var helperPos = node._startIndex;

    for (i = 0; i < node0Elements.length; i++) {
        currElement = node0Elements[i];
        BVH.copyBox(this._bboxArray, currElement, this._bboxHelper, helperPos);
        helperPos++;
    }

    for (i = 0; i < node1Elements.length; i++) {
        currElement = node1Elements[i];
        BVH.copyBox(this._bboxArray, currElement, this._bboxHelper, helperPos);
        helperPos++;
    }

    // copy results back to main array
    var subArr = this._bboxHelper.subarray(node._startIndex * 7, node._endIndex * 7);
    this._bboxArray.set(subArr, node._startIndex * 7);

    // create 2 new nodes for the node we just split, and add links to them from the parent node
    var node0Extents = this.calcExtents(node0Start, node0End);
    var node1Extents = this.calcExtents(node1Start, node1End);

    var node0 = new BVHNode(node0Extents[0], node0Extents[1], node0Start, node0End, node._level + 1);
    var node1 = new BVHNode(node1Extents[0], node1Extents[1], node1Start, node1End, node._level + 1);

    node._node0 = node0;
    node._node1 = node1;
    node.clearShapes();

    // add new nodes to the split queue
    this._nodesToSplit.push(node0);
    this._nodesToSplit.push(node1);
};

BVH.intersectNodeBox = function(rayOrigin, rayDirection, node) {
    var minX = node._extentsMin.x - 1e-6;
    var minY = node._extentsMin.y - 1e-6;
    var minZ = node._extentsMin.z - 1e-6;
    var maxX = node._extentsMax.x + 1e-6;
    var maxY = node._extentsMax.y + 1e-6;
    var maxZ = node._extentsMax.z + 1e-6;

    var tmin,tmax,tymin,tymax,tzmin,tzmax;

    var invdirx = 1 / rayDirection.x;
    var invdiry = 1 / rayDirection.y;
    var invdirz = 1 / rayDirection.z;

    if ( invdirx >= 0 ) {

        tmin = ( minX - rayOrigin.x ) * invdirx;
        tmax = ( maxX - rayOrigin.x ) * invdirx;

    } else {

        tmin = ( maxX - rayOrigin.x ) * invdirx;
        tmax = ( minX - rayOrigin.x ) * invdirx;
    }

    if ( invdiry >= 0 ) {

        tymin = ( minY - rayOrigin.y ) * invdiry;
        tymax = ( maxY - rayOrigin.y ) * invdiry;

    } else {

        tymin = ( maxY - rayOrigin.y ) * invdiry;
        tymax = ( minY - rayOrigin.y ) * invdiry;
    }

    if ( ( tmin > tymax ) || ( tymin > tmax ) ) {
        return false;
    }

    // These lines also handle the case where tmin or tmax is NaN
    // (result of 0 * Infinity). x !== x returns true if x is NaN

    if ( tymin > tmin || tmin !== tmin ) {
        tmin = tymin;
    }

    if ( tymax < tmax || tmax !== tmax ) {
        tmax = tymax;
    }

    if ( invdirz >= 0 ) {
        tzmin = ( minZ - rayOrigin.z ) * invdirz;
        tzmax = ( maxZ - rayOrigin.z ) * invdirz;
    } else {
        tzmin = ( maxZ - rayOrigin.z ) * invdirz;
        tzmax = ( minZ - rayOrigin.z ) * invdirz;
    }

    if ( ( tmin > tzmax ) || ( tzmin > tmax ) ) {
        return false;
    }

    if ( tzmin > tmin || tmin !== tmin ) {
        tmin = tzmin;
    }

    if ( tzmax < tmax || tmax !== tmax ) {
        tmax = tzmax;
    }

    //return point closest to the ray (positive side)
    if ( tmax < 0 ) {
        return false;
    }

    return true;
};

BVH.intersectRayTriangle = function () {
    // Compute the offset origin, edges, and normal.
    var diff = new BVHVector3();
    var edge1 = new BVHVector3();
    var edge2 = new BVHVector3();
    var normal = new BVHVector3();

    return function (a, b, c, rayOrigin, rayDirection, backfaceCulling) {

        // from http://www.geometrictools.com/LibMathematics/Intersection/Wm5IntrRay3Triangle3.cpp

        edge1.subVectors(b, a);
        edge2.subVectors(c, a);
        normal.crossVectors(edge1, edge2);

        // Solve Q + t*D = b1*E1 + b2*E2 (Q = kDiff, D = ray direction,
        // E1 = kEdge1, E2 = kEdge2, N = Cross(E1,E2)) by
        //   |Dot(D,N)|*b1 = sign(Dot(D,N))*Dot(D,Cross(Q,E2))
        //   |Dot(D,N)|*b2 = sign(Dot(D,N))*Dot(D,Cross(E1,Q))
        //   |Dot(D,N)|*t = -sign(Dot(D,N))*Dot(Q,N)
        var DdN = rayDirection.dot(normal);
        var sign;

        if (DdN > 0) {

            if (backfaceCulling) return null;
            sign = 1;

        } else if (DdN < 0) {

            sign = -1;
            DdN = -DdN;

        } else {

            return null;

        }

        diff.subVectors(rayOrigin, a);
        var DdQxE2 = sign * rayDirection.dot(edge2.crossVectors(diff, edge2));

        // b1 < 0, no intersection
        if (DdQxE2 < 0) {
            return null;
        }

        var DdE1xQ = sign * rayDirection.dot(edge1.cross(diff));

        // b2 < 0, no intersection
        if (DdE1xQ < 0) {
            return null;
        }

        // b1+b2 > 1, no intersection
        if (DdQxE2 + DdE1xQ > DdN) {
            return null;
        }

        // Line intersects triangle, check if ray does.
        var QdN = -sign * diff.dot(normal);

        // t < 0, no intersection
        if (QdN < 0) {
            return null;
        }

        // Ray intersects triangle.
        var t = QdN / DdN;
        var result = new THREE.Vector3();
        return result.copy( rayDirection ).multiplyScalar( t ).add( rayOrigin );
    };
}();

BVH.setBox = function(bboxArray, pos, triangleId, minX, minY, minZ, maxX, maxY, maxZ) {
    bboxArray[pos*7] = triangleId;
    bboxArray[pos*7+1] = minX;
    bboxArray[pos*7+2] = minY;
    bboxArray[pos*7+3] = minZ;
    bboxArray[pos*7+4] = maxX;
    bboxArray[pos*7+5] = maxY;
    bboxArray[pos*7+6] = maxZ;
};

BVH.copyBox = function(sourceArray, sourcePos, destArray, destPos) {
    destArray[destPos*7] = sourceArray[sourcePos*7];
    destArray[destPos*7+1] = sourceArray[sourcePos*7+1];
    destArray[destPos*7+2] = sourceArray[sourcePos*7+2];
    destArray[destPos*7+3] = sourceArray[sourcePos*7+3];
    destArray[destPos*7+4] = sourceArray[sourcePos*7+4];
    destArray[destPos*7+5] = sourceArray[sourcePos*7+5];
    destArray[destPos*7+6] = sourceArray[sourcePos*7+6];
};

BVH.getBox = function(bboxArray, pos, outputBox) {
    outputBox.triangleId = bboxArray[pos*7];
    outputBox.minX = bboxArray[pos*7+1];
    outputBox.minY = bboxArray[pos*7+2];
    outputBox.minZ = bboxArray[pos*7+3];
    outputBox.maxX = bboxArray[pos*7+4];
    outputBox.maxY = bboxArray[pos*7+5];
    outputBox.maxZ = bboxArray[pos*7+6];
};


/**
 * A node in the BVH structure
 * @param extentsMin the min coords of this node's bounding box ({x,y,z})
 * @param extentsMax the max coords of this node's bounding box ({x,y,z})
 * @param startIndex an index in the bbox array, where the first element of this node is located
 * @param endIndex an index in the bbox array, where the last of this node is located, plus 1 (meaning that its non-inclusive).
 */
function BVHNode(extentsMin, extentsMax, startIndex, endIndex, level) {
    this._extentsMin = extentsMin;
    this._extentsMax = extentsMax;
    this._boundingSphereRadius = BVHNode.calcBoundingSphereRadius(extentsMin, extentsMax);
    this._startIndex = startIndex;
    this._endIndex = endIndex;
    this._level = level;
    this._node0 = null;
    this._node1 = null;
}

BVHNode.prototype.elementCount = function() {
    return this._endIndex - this._startIndex;
};

BVHNode.prototype.centerX = function() {
    return (this._extentsMin.x + this._extentsMax.x) * 0.5;
};

BVHNode.prototype.centerY = function() {
    return (this._extentsMin.y + this._extentsMax.y) * 0.5;
};

BVHNode.prototype.centerZ = function() {
    return (this._extentsMin.z + this._extentsMax.z) * 0.5;
};

BVHNode.prototype.clearShapes = function() {
    this._startIndex = -1;
    this._endIndex = -1;
};

BVHNode.calcBoundingSphereRadius = function(extentsMin, extentsMax) {
    var centerX = (extentsMin.x + extentsMax.x) * 0.5;
    var centerY = (extentsMin.y + extentsMax.y) * 0.5;
    var centerZ = (extentsMin.z + extentsMax.z) * 0.5;

    var extentsMinDistSqr =
        (centerX - extentsMin.x) * (centerX - extentsMin.x) +
        (centerY - extentsMin.y) * (centerY - extentsMin.y) +
        (centerZ - extentsMin.z) * (centerZ - extentsMin.z);

    var extentsMaxDistSqr =
        (centerX - extentsMax.x) * (centerX - extentsMax.x) +
        (centerY - extentsMax.y) * (centerY - extentsMax.y) +
        (centerZ - extentsMax.z) * (centerZ - extentsMax.z);

    return Math.sqrt(Math.max(extentsMinDistSqr, extentsMaxDistSqr));
};
