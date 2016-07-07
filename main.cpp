#include <iostream>
#include <vector>
#include <map>


struct Point
{
	double x, y;
	Point & operator = (Point const & b) { x = b.x; y = b.y; return *this; }
	Point(const Point & b) : x(b.x), y(b.y) {}
	Point(double _x, double _y) : x(_x), y(_y) {}
	bool operator <(const Point & b) const
	{
		if (y < b.y - 1.0e-9) return true;
		else if (y > b.y + 1.0e-9) return false;
		else if (x < b.x - 1.0e-9) return true;
		else return false;
	}
	bool operator ==(const Point & b) const
	{
		return fabs(y - b.y) < 1.0e-9 && fabs(x - b.x) < 1.0e-9;
	}
	bool operator !=(const Point & b) const
	{
		return fabs(y - b.y) > 1.0e-9 || fabs(x - b.x) > 1.0e-9;
	}
};

struct Segment
{
	Point beg, end;
	Segment & operator = (Segment const & b) { beg = b.beg; end = b.end; return *this; }
	Segment(const Segment & b) : beg(b.beg), end(b.end) {}
	Segment(const Point & _beg, const Point & _end) : beg(_beg), end(_end) {}
};

enum EventType
{
	Start,
	End
};

struct Event
{
	EventType type;
	int segment;
	Event(EventType _type, int _segment) : type(_type), segment(_segment) {}
};


std::pair<bool,Point> intersect(const Segment & a, const Segment & b)
{
	Point ret(0, 0);
	double div = (a.beg.x - a.end.x)*(b.beg.y - b.end.y) - (a.beg.y - a.end.y)*(b.beg.x - b.end.x), t;
	if (fabs(div) < 1.0e-13) 
		return std::make_pair(false, ret);
	ret.x = ((a.beg.x*a.end.y - a.beg.y*a.end.x)*(b.beg.x - b.end.x) - (a.beg.x - a.end.x)*(b.beg.x*b.end.y - b.beg.y*b.end.x)) / div;
	ret.y = ((a.beg.x*a.end.y - a.beg.y*a.end.x)*(b.beg.y - b.end.y) - (a.beg.y - a.end.y)*(b.beg.x*b.end.y - b.beg.y*b.end.x)) / div;
	t = (ret.x - a.beg.x) / (a.end.x - a.beg.x);
	if (t < 1.0e-9 || t > 1.0 - 1.0e-9) return std::make_pair(false, ret);
	t = (ret.y - a.beg.y) / (a.end.y - a.beg.y);
	if (t < 1.0e-9 || t > 1.0 - 1.0e-9) return std::make_pair(false, ret);
	t = (ret.x - b.beg.x) / (b.end.x - b.beg.x);
	if (t < 1.0e-9 || t > 1.0 - 1.0e-9) return std::make_pair(false, ret);
	t = (ret.y - b.beg.y) / (b.end.y - b.beg.y);
	if (t < 1.0e-9 || t > 1.0 - 1.0e-9) return std::make_pair(false, ret);
	return std::make_pair(true, ret);
}

void intersect(int a, int b, const Point & I, std::vector<Segment> & segments, std::multimap<Point, int> & sweep, std::multimap<double, Event> & events)
{
	//remove event of ending of old segment
	{
		int rem_end_events[2];
		rem_end_events[0] = a;
		rem_end_events[1] = b;
		for (int k = 0; k < 2; ++k)
		{
			std::pair< std::multimap<double, Event>::iterator, std::multimap<double, Event>::iterator > del = events.equal_range(segments[rem_end_events[k]].end.x); //get all events at position of the end
			bool flag = false;
			for (std::multimap<double, Event>::iterator it = del.first; it != del.second; ++it) //search over all events
			{
				if (it->second.type == End && it->second.segment == rem_end_events[k]) //event is end of segment and segment matches current
				{
					events.erase(it); //remove that segment
					flag = true;
					break; //do not expect any more
				}
			}
			if (!flag) std::cout << "Cannot find proper ending event for segment" << std::endl;
		}
	}
	//add new segment with intersection point up to end
	segments.push_back(Segment(I, segments[a].end));
	//add event of starting of new segment
	events.insert(std::make_pair(I.x, Event(Start, (int)segments.size() - 1)));
	//add event of ending of new segment
	events.insert(std::make_pair(segments.back().end.x, Event(End, (int)segments.size() - 1)));
	//change ending point for current segment
	segments[a].end = I;
	//add event of ending of old segment
	events.insert(std::make_pair(I.x, Event(End, a)));
	//add new segment with intersection point up to end
	segments.push_back(Segment(I, segments[b].end));
	//add event of starting of new segment
	events.insert(std::make_pair(I.x, Event(Start, (int)segments.size() - 1)));
	//add event of ending of new segment
	events.insert(std::make_pair(segments.back().end.x, Event(End, (int)segments.size() - 1)));
	//change ending point for current segment
	segments[b].end = I;
	//add event of ending of old segment
	events.insert(std::make_pair(I.x, Event(End, b)));
}

//find all intersection points
void intersect(std::vector<Segment> & segments, std::vector<Point> & intersections)
{
	std::multimap<double,Event> events;
	std::multimap<Point,int> sweep;


	for (int k = 0; k < (int)segments.size(); ++k)
	{
		if (segments[k].beg.x > segments[k].end.x)
			std::swap(segments[k].beg, segments[k].end);
		events.insert(std::make_pair(segments[k].beg.x,Event(Start, k)));
		events.insert(std::make_pair(segments[k].end.x,Event(End, k)));
	}


	while (!events.empty())
	{
		std::multimap<double,Event>::iterator first = events.begin();
		Event e = first->second;
		events.erase(first);
		int s = e.segment;
		if (e.type == Start)
		{
			//check if there is a line with same position
			std::multimap<Point, int>::iterator ins = sweep.insert(std::make_pair(segments[s].beg, s));
			//check line (or lines above current)
			for (int dir = 0; dir <= 1; ++dir) // look up or down
			{
				std::multimap<Point, int>::iterator iter = ins;
				while ((dir ? ++iter : --iter) != sweep.end()) //y is greater for next
				{
					if (segments[s].beg != segments[iter->second].beg) //ignore same starting position
					{
						std::pair<bool, Point> I = intersect(segments[s], segments[iter->second]);
						if (I.first)
						{
							intersections.push_back(I.second);
							intersect(s, iter->second, I.second, segments, sweep, events);
						}
					}
					if ((2*dir-1)*(iter->first.y - ins->first.y) > 0) //visited line is above (below) current
						break; //stop search
				}
			}
		}
		else if (e.type == End)
		{
			//remove segment from sweep
			std::pair< std::multimap<Point, int>::iterator, std::multimap<Point, int>::iterator > del = sweep.equal_range(segments[s].beg);
			std::multimap<Point, int>::iterator above, below;
			bool flag = false;
			for (std::multimap<Point, int>::iterator it = del.first; it != del.second; ++it) //search over all events
			{
				if( it->second == s) //found necessery segment
				{
					above = it; ++above;
					below = it; --below;
					sweep.erase(it); //remove that segment
					flag = true;
					break; //do not expect any more
				}
			}
			if (!flag) std::cout << "Error: cannot find segment in sweep" << std::endl;
			else
			{
				if (above != sweep.end() && below != sweep.end())
				{
					if (segments[above->second].beg != segments[below->second].beg)
					{
						std::pair<bool, Point> I = intersect(segments[below->second], segments[above->second]);
						if (I.first)
						{
							intersections.push_back(I.second);
							intersect(below->second, above->second, I.second, segments, sweep, events);

						}
					}
				}
			}
		}
	}
	
}


int main()
{
	std::vector<Point> intersections;
	std::vector<Segment> segments;
	segments.push_back(Segment(Point(-7.41, -0.58), Point(-1.3,-0.79)));
	segments.push_back(Segment(Point(-4.0, 1.27), Point(-4.21, -2.99)));
	segments.push_back(Segment(Point(-4.92, 0.71), Point(-4.26, -1.40)));
	segments.push_back(Segment(Point(-4.55, -1.24), Point(-2.54, -0.42)));
	intersect(segments, intersections);
	std::cout << "Intersection points[" << intersections.size() << "]: " << std::endl;
	for (std::vector<Point>::iterator it = intersections.begin(); it != intersections.end(); ++it)
		std::cout << "(" << it->x << "," << it->y << ") " << std::endl;
	std::cout << std::endl;
	std::cout << "Segments[" << segments.size() << "]: " << std::endl;
	for (std::vector<Segment>::iterator it = segments.begin(); it != segments.end(); ++it)
		std::cout << "[ (" << it->beg.x << "," << it->beg.y << "), (" << it->end.x << "," << it->end.y << ") ] " << std::endl;
	return 0;
}