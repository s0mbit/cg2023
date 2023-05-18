////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
//////////////////////////////////////////////////////////////////////////////// 
using namespace std;

const string root_path = "/home/sk/CG-Summer-2023/Assignment_1/data";

typedef complex<double> Point;
typedef vector<Point> Polygon;

double inline det(const Point &u, const Point &v)
{
    return (u.real() * v.imag() - u.imag() * v.real());
}

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &ans)
{
    // Compute vectors
    Point ab = b - a;
    Point cd = d - c;
    Point ac = c - a;

    // Compute cross products
    double ab_cross_cd = det(ab,cd);
    double ac_cross_cd = det(ac,cd);
    double ac_cross_ab = det(ac,ab);

    // Check if parallel
    if (ab_cross_cd == 0) {
        return false;
    }

    double t1 = ac_cross_cd / ab_cross_cd;
    double t2 = ac_cross_ab / ab_cross_cd;

    if (t1 < 0 || t1 > 1 || t2 < 0) {
        return false;
    }

    ans = a + t1 * ab;
    return true;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query)
{
    // Hardcoded point! might be better if generic but who really cares
    Point outside(6000,6000);

    // 2. Cast a ray from the query point to the 'outside' point, count number of intersections
    int count = 0;
    for (int i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        Point intersection;
        //checks if the vector from i to j is coliding with the query to outside ray.
        if (intersect_segment(poly[i], poly[j], outside, query, intersection)) {
            if (intersection == query) {
                cout << "inter == query" << endl;
                return true;
            }
            if ((intersection.real() > query.real()) != (outside.real() > query.real())) {
                count++;
            }
        }
    }
    //cout << count << endl;
    if (count % 2 == 1) {
        return true;
    } else {
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////

double distance_squared(const Point &p1, const Point &p2)
{
    double dx = p1.real() - p2.real();
    double dy = p1.imag() - p2.imag();
    return dx * dx + dy * dy;
}

struct Compare
{
    Point p0; // Leftmost point of the poly
    bool operator()(const Point &p1, const Point &p2)
    {
        auto cross = det(p1 - p0, p2 - p0);
        if (cross == 0)
        {
            auto d1 = distance_squared(p1, p0);
            auto d2 = distance_squared(p2, p0);
            return d1 < d2;
        }
        return cross > 0;
    }
};

bool inline salientAngle(const Point &a, const Point &b, const Point &c)
{
    auto cross = det(b - a, c - a);
    return cross > 0;
}

Polygon convex_hull(vector<Point> &points)
{
    Point leftmostPoint = *min_element(points.begin(), points.end(),
                                       [](const Point &p1, const Point &p2) { return p1.real() < p2.real(); });

    auto compare = [&leftmostPoint](const Point &p1, const Point &p2)
    {
        auto cross = det(p1 - leftmostPoint, p2 - leftmostPoint);
        if (cross == 0)
        {
            auto d1 = distance_squared(p1, leftmostPoint);
            auto d2 = distance_squared(p2, leftmostPoint);
            return d1 < d2;
        }
        return cross > 0;
    };

    sort(points.begin(), points.end(), compare);

    Polygon hull;
    hull.push_back(points[0]);
    hull.push_back(points[1]);

    for (size_t i = 2; i < points.size(); ++i)
    {
        while (hull.size() >= 2 && !salientAngle(*(hull.rbegin() + 1), hull.back(), points[i]))
        {
            hull.pop_back();
        }

        hull.push_back(points[i]);
    }
    return hull;
} 

////////////////////////////////////////////////////////////////////////////////

vector<Point> load_xyz(const string &filename)
{
    vector<Point> points;
    ifstream in(filename);
    // TODO
    size_t num_points;
    in >> num_points;
    double x, y, z;
    for (size_t i = 0; i < num_points; ++i) {
        in >> x >> y >> z;
        points.push_back(Point(x, y));
    }
    return points;
}

void save_xyz(const string &filename, const vector<Point> &points)
{
    ofstream out(filename);
    out << points.size() << '\n';

    for (const auto &p : points)
    {
        out << p.real() << ' ' << p.imag() << " 0\n";
    }
    out.close();
}


Polygon load_obj(const std::string& filename) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        throw std::runtime_error("failed to open file " + filename);
    }

    Polygon poly;
    string line;
    while (getline(in, line)) {
        istringstream iss(line);
        string type;
        iss >> type;
        if (type == "v") {
            double x, y, z;
            iss >> x >> y >> z;
            poly.emplace_back(x, y);
        } else if (type == "f") {
            string index_str;
            while (iss >> index_str) {
                int index = stoi(index_str) - 1;
            }
        } else {
            throw std::runtime_error("error?");
        }
    }

    return poly;
}

void save_obj(const string &filename, Polygon &poly)
{
    ofstream out(filename);
    if (!out.is_open())
    {
        throw runtime_error("failed to open file " + filename);
    }
    out << fixed;
    for (const auto &v : poly)
    {
        out << "v " << v.real() << ' ' << v.imag() << " 0\n";
    }
    for (size_t i = 0; i < poly.size(); ++i)
    {
        out << "l " << i + 1 << ' ' << 1 + (i + 1) % poly.size() << "\n";
    }
    out << endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    cout << "starting.." << endl;
    

    const string points_path = root_path + "/points.xyz";
    const string poly_path = root_path + "/polygon.obj";

    vector<Point> points = load_xyz(points_path);

    if (points.empty()) {
        cerr << "No points loaded from file: " << points_path << endl;
        return 1;
    }

    ////////////////////////////////////////////////////////////////////////////////
    //Point in polygon
    Polygon poly = load_obj(poly_path);
    vector<Point> result;
    cout << points.size() << endl;
    for (size_t i = 0; i < points.size(); ++i)
    {
        //cout << "loop" << endl;
        if (is_inside(poly, points[i]))
        {
            result.push_back(points[i]);
            cout << "Added point " << points[i] << endl;
            
        }
    }
    save_xyz("output.xyz", result);

    ////////////////////////////////////////////////////////////////////////////////
    //Convex hull
    cout << "convex hull" << endl;
    Polygon hull = convex_hull(points);

    cout << "finished hull" << endl;

    save_obj("output.obj", hull);

    cout << "finished obj save" << endl;

    return 0;
}
