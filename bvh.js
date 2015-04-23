/**
 * Bounding Volume Hierarchy data structure
 */

function BVH(trianglesArray, maxTrianglesPerNode) {
    this._trianglesArray = trianglesArray;
    this._maxTrianglesPerNode = maxTrianglesPerNode;
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
 * Gets an array of triangle, and calculates the bounding box for each of them, and adds an index to the triangle's position in the array
 * Each bbox is saved as 7 values in a Float32Array: (position, minX, minY, minZ, maxX, maxY, maxZ)
 */
BVH.prototype.calcBoundingBoxes = function(trianglesArray) {
    var p0x, p0y, p0z;
    var p1x, p1y, p1z;
    var p2x, p2y, p2z;

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

        bboxArray[i*7] = i;
        bboxArray[i*7+1] = Math.min(Math.min(p0x, p1x), p2x);
        bboxArray[i*7+2] = Math.min(Math.min(p0y, p1y), p2y);
        bboxArray[i*7+3] = Math.min(Math.min(p0z, p1z), p2z);
        bboxArray[i*7+4] = Math.max(Math.max(p0x, p1x), p2x);
        bboxArray[i*7+5] = Math.max(Math.max(p0y, p1y), p2y);
        bboxArray[i*7+6] = Math.max(Math.max(p0z, p1z), p2z);
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

    //console.log("splitNode: node level is ",node._level);
    //console.log("splitNode: node extentsMin ",node._extentsMin);
    //console.log("splitNode: node extentsMax ",node._extentsMax);

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
        //console.log("splitNode: couldnt split node by any axis");
        return;
    }

    // in case split succeeded, choose to split by the axis which resulted in the most balanced split of shapes between the two child nodes
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

    // sort the elements in range (startIndex, endIndex) according to which node they should be at
    var node0Start = startIndex;
    var node0End = node0Start + node0Elements.length;
    var node1Start = node0End;
    var node1End = endIndex;
    var currElement, pos;

    var helperPos = node._startIndex;

    for (i = 0; i < node0Elements.length; i++) {
        currElement = node0Elements[i];
        this._bboxHelper[helperPos*7] = this._bboxArray[currElement*7];
        this._bboxHelper[helperPos*7+1] = this._bboxArray[currElement*7+1];
        this._bboxHelper[helperPos*7+2] = this._bboxArray[currElement*7+2];
        this._bboxHelper[helperPos*7+3] = this._bboxArray[currElement*7+3];
        this._bboxHelper[helperPos*7+4] = this._bboxArray[currElement*7+4];
        this._bboxHelper[helperPos*7+5] = this._bboxArray[currElement*7+5];
        this._bboxHelper[helperPos*7+6] = this._bboxArray[currElement*7+6];
        helperPos++;
    }

    for (i = 0; i < node1Elements.length; i++) {
        currElement = node1Elements[i];
        this._bboxHelper[helperPos*7] = this._bboxArray[currElement*7];
        this._bboxHelper[helperPos*7+1] = this._bboxArray[currElement*7+1];
        this._bboxHelper[helperPos*7+2] = this._bboxArray[currElement*7+2];
        this._bboxHelper[helperPos*7+3] = this._bboxArray[currElement*7+3];
        this._bboxHelper[helperPos*7+4] = this._bboxArray[currElement*7+4];
        this._bboxHelper[helperPos*7+5] = this._bboxArray[currElement*7+5];
        this._bboxHelper[helperPos*7+6] = this._bboxArray[currElement*7+6];
        helperPos++;
    }

    //console.log("SPLIT STATISTICS");
    //console.log("startIndex: ",node._startIndex," endIndex: ",node._endIndex, " delta: ",node._endIndex - node._startIndex);
    //console.log("helper index after iteration: ",helperPos);
    //console.log("subArr length: ",(node0Elements.length + node1Elements.length));
    //console.log("node0 start: ",node0Start, "node0 End: ",node0End);
    //console.log("node1 start: ",node1Start, "node1 End: ",node1End);

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
