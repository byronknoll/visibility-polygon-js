/*
This code is released into the public domain - attribution is appreciated but not required.
Made by Byron Knoll in 2013.

https://code.google.com/p/visibility-polygon-js/
Demo: http://www.byronknoll.com/visibility.html

This library can be used to construct a visibility polygon for a set of line segments.

The time complexity of this implementation is O(N log N) (where N is the total number of line segments). This is the optimal time complexity for this problem.

The following three functions should be useful:

1) VisibilityPolygon.compute(position, segments)
  Computes a visibility polygon. O(N log N) time complexity.
  Arguments:
    position - The location of the observer. Must be completely surrounded by line segments (an easy way to enforce this is to create an outer bounding box).
    segments - A list of line segments. Each line segment should be a list of two points. Each point should be a list of two coordinates. Line segments can *not* intersect each other (although overlapping vertices is OK).
  Returns: The visibility polygon (in clockwise vertex order).

2) VisibilityPolygon.inPolygon(position, polygon)
  Calculates whether a point is within a polygon. O(N) time complexity.
  Arguments:
    position - The point to check: a list of two coordinates.
    polygon - The polygon to check: a list of points. The polygon can be specified in either clockwise or counterclockwise vertex order.
  Returns: True if "position" is within the polygon.

3) VisibilityPolygon.convertToSegments(polygons)
  Converts the given polygons to list of line segments. O(N) time complexity.
  Arguments: a list of polygons (in either clockwise or counterclockwise vertex order). Each polygon should be a list of points. Each point should be a list of two coordinates.
  Returns: a list line segments.

Example code:

var polygons = [];
polygons.push([[-1,-1],[501,-1],[501,501],[-1,501]]);
polygons.push([[250,100],[260,140],[240,140]]);
var segments = VisibilityPolygon.convertToSegments(polygons);
var position = [10, 10];
if (VisibilityPolygon.inPolygon(position, polygons[0])) {
  var visibility = VisibilityPolygon.compute(position, segments);
}

*/

function VisibilityPolygon(){};

VisibilityPolygon.compute = function(position, segments) {
	var polygon = [];
	var sorted = VisibilityPolygon.sortPoints(position, segments);
	var map = new Array(segments.length);
	for (var i = 0; i < map.length; ++i) map[i] = -1;
	var heap = [];
	var start = [position[0] + 1, position[1]];
	for (var i = 0; i < segments.length; ++i) {
		var a1 = VisibilityPolygon.angle(segments[i][0], position);
		var a2 = VisibilityPolygon.angle(segments[i][1], position);
		var active = false;
		if (a1 > -180 && a1 <= 0 && a2 <= 180 && a2 >= 0 && a2 - a1 > 180) active = true;
		if (a2 > -180 && a2 <= 0 && a1 <= 180 && a1 >= 0 && a1 - a2 > 180) active = true;
		if (active) {
			VisibilityPolygon.insert(i, heap, position, segments, start, map);
		}
	}
	for (var i = 0; i < sorted.length;) {
		var extend = false;
		var shorten = false;
		var orig = i;
		var vertex = segments[sorted[i][0]][sorted[i][1]];
		var old_segment = heap[0];
		do {
			if (map[sorted[i][0]] != -1) {
				if (sorted[i][0] == old_segment) {
					extend = true;
					vertex = segments[sorted[i][0]][sorted[i][1]];
				}
				VisibilityPolygon.remove(map[sorted[i][0]], heap, position, segments, vertex, map);
			} else {
				VisibilityPolygon.insert(sorted[i][0], heap, position, segments, vertex, map);
				if (heap[0] != old_segment) {
					shorten = true;
				}
			}
			++i;
			if (i == sorted.length) break;
		} while (sorted[i][2] < sorted[orig][2] + VisibilityPolygon.epsilon());

		if (extend) {
			polygon.push(vertex);
			var cur = VisibilityPolygon.intersectLines(segments[heap[0]][0], segments[heap[0]][1], position, vertex);
			if (!VisibilityPolygon.equal(cur, vertex)) polygon.push(cur);
		} else if (shorten) {
			polygon.push(VisibilityPolygon.intersectLines(segments[old_segment][0], segments[old_segment][1], position, vertex));
			polygon.push(VisibilityPolygon.intersectLines(segments[heap[0]][0], segments[heap[0]][1], position, vertex));
		} 
	}
	return polygon;
};

VisibilityPolygon.inPolygon = function(position, polygon) {
	var val = 0;
	for (var i = 0; i < polygon.length; ++i) {
		val = Math.min(polygon[i][0], val);
		val = Math.min(polygon[i][1], val);
	}
	var edge = [val-1, val-1];
	var parity = 0;
	for (var i = 0; i < polygon.length; ++i) {
		var j = i + 1;
		if (j == polygon.length) j = 0;
		if (VisibilityPolygon.doLineSegmentsIntersect(edge[0], edge[1], position[0], position[1], polygon[i][0], polygon[i][1], polygon[j][0], polygon[j][1])) {
			var intersect = VisibilityPolygon.intersectLines(edge, position, polygon[i], polygon[j]);
			if (VisibilityPolygon.equal(position, intersect)) return true;
			if (VisibilityPolygon.equal(intersect, polygon[i])) {
				if (VisibilityPolygon.angle2(position, edge, polygon[j]) < 180) ++parity;
			} else if (VisibilityPolygon.equal(intersect, polygon[j])) {
				if (VisibilityPolygon.angle2(position, edge, polygon[i]) < 180) ++parity;
			} else {
				++parity;
			}
		}
	}
	return (parity%2)!=0;
};

VisibilityPolygon.convertToSegments = function(polygons) {
	var segments = [];
	for (var i = 0; i < polygons.length; ++i) {
		for (var j = 0; j < polygons[i].length; ++j) {
			var k = j+1;
			if (k == polygons[i].length) k = 0;
			segments.push([polygons[i][j], polygons[i][k]]);
		}
	}
	return segments;
};

VisibilityPolygon.epsilon = function() {
	return 0.0000001;
};

VisibilityPolygon.equal = function(a, b) {
	if (Math.abs(a[0] - b[0]) < VisibilityPolygon.epsilon() && Math.abs(a[1] - b[1]) < VisibilityPolygon.epsilon()) return true;
	return false;
};

VisibilityPolygon.remove = function(index, heap, position, segments, destination, map) {
	map[heap[index]] = -1;
	if (index == heap.length - 1) {
		heap.pop();
		return;
	}
	heap[index] = heap.pop();
	map[heap[index]] = index;
	var cur = index;
	var parent = VisibilityPolygon.parent(cur);
	if (cur != 0 && VisibilityPolygon.lessThan(heap[cur], heap[parent], position, segments, destination)) {
		while (cur > 0) {
			var parent = VisibilityPolygon.parent(cur);
			if (!VisibilityPolygon.lessThan(heap[cur], heap[parent], position, segments, destination)) {
				break;
			}
			map[heap[parent]] = cur;
			map[heap[cur]] = parent;
			var temp = heap[cur];
			heap[cur] = heap[parent];
			heap[parent] = temp;
			cur = parent;
		}
	} else {
		while (true) {
			var left = VisibilityPolygon.child(cur);
			var right = left + 1;
			if (left < heap.length && VisibilityPolygon.lessThan(heap[left], heap[cur], position, segments, destination) &&
					(right == heap.length || VisibilityPolygon.lessThan(heap[left], heap[right], position, segments, destination))) {
				map[heap[left]] = cur;
				map[heap[cur]] = left;
				var temp = heap[left];
				heap[left] = heap[cur];
				heap[cur] = temp;
				cur = left;
			} else if (right < heap.length && VisibilityPolygon.lessThan(heap[right], heap[cur], position, segments, destination)) {
				map[heap[right]] = cur;
				map[heap[cur]] = right;
				var temp = heap[right];
				heap[right] = heap[cur];
				heap[cur] = temp;
				cur = right;
			} else break;
		}
	}
};

VisibilityPolygon.insert = function(index, heap, position, segments, destination, map) {
	var intersect = VisibilityPolygon.intersectLines(segments[index][0], segments[index][1], position, destination);
	if (intersect.length == 0) return;
	var cur = heap.length;
	heap.push(index);
	map[index] = cur;
	while (cur > 0) {
		var parent = VisibilityPolygon.parent(cur);
		if (!VisibilityPolygon.lessThan(heap[cur], heap[parent], position, segments, destination)) {
			break;
		}
		map[heap[parent]] = cur;
		map[heap[cur]] = parent;
		var temp = heap[cur];
		heap[cur] = heap[parent];
		heap[parent] = temp;
		cur = parent;
	}
};

VisibilityPolygon.lessThan = function(index1, index2, position, segments, destination) {
	var inter1 = VisibilityPolygon.intersectLines(segments[index1][0], segments[index1][1], position, destination);
	var inter2 = VisibilityPolygon.intersectLines(segments[index2][0], segments[index2][1], position, destination);
	if (!VisibilityPolygon.equal(inter1, inter2)) {
		var d1 = VisibilityPolygon.distance(inter1, position);
		var d2 = VisibilityPolygon.distance(inter2, position);
		return d1 < d2;
	}
	var end1 = 0;
	if (VisibilityPolygon.equal(inter1, segments[index1][0])) end1 = 1;
	var end2 = 0;
	if (VisibilityPolygon.equal(inter2, segments[index2][0])) end2 = 1;
	var a1 = VisibilityPolygon.angle2(segments[index1][end1], inter1, position);
	var a2 = VisibilityPolygon.angle2(segments[index2][end2], inter2, position);
	if (a1 < 180) {
		if (a2 > 180) return true;
		return a2 < a1;
	}
	return a1 < a2;
};

VisibilityPolygon.parent = function(index) {
	return Math.floor((index-1)/2);
};

VisibilityPolygon.child = function(index) {
	return 2*index+1;
};

VisibilityPolygon.angle2 = function(a, b, c) {
	var a1 = VisibilityPolygon.angle(a,b);
	var a2 = VisibilityPolygon.angle(b,c);
	var a3 = a1 - a2;
	if (a3 < 0) a3 += 360;
	if (a3 > 360) a3 -= 360;
	return a3;
};

VisibilityPolygon.sortPoints = function(position, segments) {
	var points = new Array(segments.length * 2);
	for (var i = 0; i < segments.length; ++i) {
		for (var j = 0; j < 2; ++j) {
			var a = VisibilityPolygon.angle(segments[i][j], position);
			points[2*i+j] = [i, j, a];
		}
	}
	points.sort(function(a,b) {return a[2]-b[2];});
	return points;
};

VisibilityPolygon.angle = function(a, b) {
	return Math.atan2(b[1]-a[1], b[0]-a[0]) * 180 / Math.PI;
};

VisibilityPolygon.intersectLines = function(a1, a2, b1, b2) {
	var ua_t = (b2[0] - b1[0]) * (a1[1] - b1[1]) - (b2[1] - b1[1]) * (a1[0] - b1[0]);
	var ub_t = (a2[0] - a1[0]) * (a1[1] - b1[1]) - (a2[1] - a1[1]) * (a1[0] - b1[0]);
	var u_b  = (b2[1] - b1[1]) * (a2[0] - a1[0]) - (b2[0] - b1[0]) * (a2[1] - a1[1]);
	if (u_b != 0) {
		var ua = ua_t / u_b;
		var ub = ub_t / u_b;
		return [a1[0] - ua * (a1[0] - a2[0]), a1[1] - ua * (a1[1] - a2[1])];
	}
	return [];
};

VisibilityPolygon.distance = function(a, b) {
	return (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]);
};

VisibilityPolygon.isOnSegment = function(xi, yi, xj, yj, xk, yk) {
  return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) &&
         (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
};

VisibilityPolygon.computeDirection = function(xi, yi, xj, yj, xk, yk) {
  a = (xk - xi) * (yj - yi);
  b = (xj - xi) * (yk - yi);
  return a < b ? -1 : a > b ? 1 : 0;
};

VisibilityPolygon.doLineSegmentsIntersect = function(x1, y1, x2, y2, x3, y3, x4, y4) {
  d1 = VisibilityPolygon.computeDirection(x3, y3, x4, y4, x1, y1);
  d2 = VisibilityPolygon.computeDirection(x3, y3, x4, y4, x2, y2);
  d3 = VisibilityPolygon.computeDirection(x1, y1, x2, y2, x3, y3);
  d4 = VisibilityPolygon.computeDirection(x1, y1, x2, y2, x4, y4);
  return (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
          ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) ||
         (d1 == 0 && VisibilityPolygon.isOnSegment(x3, y3, x4, y4, x1, y1)) ||
         (d2 == 0 && VisibilityPolygon.isOnSegment(x3, y3, x4, y4, x2, y2)) ||
         (d3 == 0 && VisibilityPolygon.isOnSegment(x1, y1, x2, y2, x3, y3)) ||
         (d4 == 0 && VisibilityPolygon.isOnSegment(x1, y1, x2, y2, x4, y4));
};
