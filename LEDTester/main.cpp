#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>

using namespace std;

    struct Point2f
    {
        float x;
        float y;
        bool operator == (const Point2f &b) const
        {
            if (x==b.x && y==b.y)
                return true;
            else
                return false;
        }
        Point2f()
        {
            x = 0;
            y = 0;
        }
        Point2f(int a,int b)
        {
            x = a;
            y = b;
        }
    };

    struct proximity
    {
        struct Point2f center;
        float radius;
        bool isDuplicate;
        bool delete_me;
        bool operator < (const proximity& b) const
        {
            return (radius < b.radius);
        }
        proximity()
        {
            center = Point2f();
            radius = 0;
            isDuplicate = false;
            delete_me = false;
        }
    };

    static bool samePoint(const proximity &a, const proximity &b)
    {
        return (a.center == b.center);
    }

    static bool compByRadius(const proximity* a, const proximity* b)
    {
        return (a->radius < b->radius);
    }

int main()
{

    const int NO_LEDS = 5;

    /// Generate some fake data

    struct proximity prox;
    vector<proximity> tmpProx;
    vector<vector<proximity> > proximityVec;
    proximityVec.resize(NO_LEDS);

    /// Group 0
    // Point 1
    prox.center.x = 0;
    prox.center.y = 0;
    prox.radius = 4;
    proximityVec[0].push_back(prox);

    // Point 2
    prox.center.x = 5;
    prox.center.y = 0;
    prox.radius = 1;
    proximityVec[0].push_back(prox);

    /// Group 1
    // Point 3
    prox.center.x = 0;
    prox.center.y = 0;
    prox.radius = 2;
    proximityVec[1].push_back(prox);

    // Point 4
    prox.center.x = 0;
    prox.center.y = 5;
    prox.radius = 0;
    proximityVec[1].push_back(prox);

    /// Group 2
    // (nothing)
    // proximityVec[2].push_back();

    /// Group 3
    // Point 5
    prox.center.x = 1;
    prox.center.y = 3;
    prox.radius = 2;
    proximityVec[3].push_back(prox);

    /// Group 4
    //(nothing)
    //proximityVec[4].push_back();

    vector<Point2f> reprojImgPts;
    reprojImgPts.push_back(Point2f(1,1));
    reprojImgPts.push_back(Point2f(2,2));
    reprojImgPts.push_back(Point2f(3,3));
    reprojImgPts.push_back(Point2f(4,4));
    reprojImgPts.push_back(Point2f(5,5));


    cout << "DATA POINTS:" << endl;
    for (int i=0; i<NO_LEDS; i++)
    {
        cout << " Group " << i << endl;
        for (int j=0; j<proximityVec[i].size(); j++)
        {
            cout << "  Prox " << j << endl;
            cout << "   center: " << proximityVec[i][j].center.x << " " << proximityVec[i][j].center.y << endl;
            cout << "   radius: " << proximityVec[i][j].radius << endl;
        }
    }


    /// Flatten vector
    vector<proximity*> flat;
    for (int i=0; i<NO_LEDS; i++)
    {
        for (int j=0; j<proximityVec[i].size(); j++)
        {
            flat.push_back(&proximityVec[i][j]);
        }
    }

    cout << endl << "FLATTENED" << endl;
    for (int i=0; i<flat.size(); i++)
    {
        cout << " Point No: " << i << endl;
        cout << "   center: " << flat[i]->center.x << " " << flat[i]->center.y << endl;
        cout << "   radius: " << flat[i]->radius << endl;
    }



    /// Detect and "group" duplicates
    vector<vector<proximity*> > groups;
    for (int i=0; i<flat.size(); i++)
    {
        if (flat[i]->isDuplicate > 0)
            continue;

        vector<proximity*> subgroup;
        subgroup.push_back(flat[i]);
        for (int j=i+1; j<flat.size(); j++)
        {
            if (samePoint(*flat[i], *flat[j]))
            {
                flat[i]->isDuplicate = true;
                flat[j]->isDuplicate = true;
                subgroup.push_back(flat[j]);
            }
        }
        if (subgroup.size() > 1)
            groups.push_back(subgroup);
    }


    cout << endl << "GROUPED DATA" << endl;
    for (int i=0; i<groups.size(); i++)
    {
        cout << " Group: " << i << endl;
        for (int j=0; j<groups[i].size(); j++)
        {
            cout << "  Point No: " << j << endl;
            cout << "   center: " << (groups[i][j]->center.x) << " " << (groups[i][j]->center.y) << endl;;
            cout << "   radius: " << (groups[i][j]->radius) << endl;
        }
    }

    /// Sort subgroups by (increasing) radius and keep only the one with the smallest radius
    for (int i=0; i<groups.size(); i++)
    {
        sort(groups[i].begin(),groups[i].end(),compByRadius);
    }

    cout << endl << "GROUPED DATA AFTER SORTING BY RADIUS" << endl;
    for (int i=0; i<groups.size(); i++)
    {
        cout << " Group: " << i << endl;
        for (int j=0; j<groups[i].size(); j++)
        {
            cout << "  Point No: " << j << endl;
            cout << "   center: " << (groups[i][j]->center.x) << " " << (groups[i][j]->center.y) << endl;;
            cout << "   radius: " << (groups[i][j]->radius) << endl;
        }
    }

    /// Eliminate any duplicate points (keeping the one(s) with the lowest radius)
    for (int i=0; i<groups.size(); i++)
    {
        for (int j=1; j<groups[i].size(); j++) // erase everything after the first element
        {
            // set flag to delete
            groups[i][j]-> delete_me = true;
        }
    }
    for (int i=0; i<proximityVec.size(); i++)
    {
        for (int j=0; j<proximityVec[i].size(); j++)
        {
            if (proximityVec[i][j].delete_me)
                proximityVec[i].erase(proximityVec[i].begin() + j);
        }
    }

    cout << endl << "DUPLICATES REMOVED" << endl;
    for (int i=0; i<NO_LEDS; i++)
    {
        cout << " Group " << i << endl;
        for (int j=0; j<proximityVec[i].size(); j++)
        {
            cout << "  Prox " << j << endl;
            cout << "   center: " << proximityVec[i][j].center.x << " " << proximityVec[i][j].center.y << endl;
            cout << "   radius: " << proximityVec[i][j].radius << endl;
        }
    }

    /// Sort by radius
    for (int i=0; i<NO_LEDS; i++)
        sort(proximityVec[i].begin(),proximityVec[i].end());  // sort by increasing radius

    cout << endl << "SORTED BY RADIUS (DUPLICATES REMOVED)" << endl;
    for (int i=0; i<NO_LEDS; i++)
    {
        cout << " Group " << i << endl;
        for (int j=0; j<proximityVec[i].size(); j++)
        {
            cout << "  Prox " << j << endl;
            cout << "   center: " << proximityVec[i][j].center.x << " " << proximityVec[i][j].center.y << endl;
            cout << "   radius: " << proximityVec[i][j].radius << endl;
        }
    }


    /// Replace any "missing" points with reprojImgPts from last time
    for (int i=0; i<NO_LEDS; i++)
    {
        if (proximityVec[i].empty())
        {
            proximity tmp;
            tmp.center = reprojImgPts[i];
            proximityVec[i].push_back(tmp);
        }
    }

    cout << endl << "SUBSTITUTING WITH REPROJIMGPTS" << endl;
    for (int i=0; i<NO_LEDS; i++)
    {
        cout << " Group " << i << endl;
        for (int j=0; j<proximityVec[i].size(); j++)
        {
            cout << "  Prox " << j << endl;
            cout << "   center: " << proximityVec[i][j].center.x << " " << proximityVec[i][j].center.y << endl;
            cout << "   radius: " << proximityVec[i][j].radius << endl;
        }
    }


    /// Return the first 5 points in order, then add points in successive layers (selected randomly) until we reach maximum (N) points
    const int LEDs_max = 8;

    vector<Point2f> LEDs;
    // Pass the first LEDs
    for (int i=0; i<NO_LEDS; i++)
        LEDs.push_back(proximityVec[i][0].center);

    // Go along successive layers to add "extra" points choosing which to pass by random
    vector<Point2f*> layer;
    int layer_no = 0;
    while (LEDs.size() < LEDs_max)
    {

        // Populate the next layer if we have extracted all elements from previous one
        if (layer.empty())
        {
            layer_no++;
            for (int i=0; i<NO_LEDS; i++)
            {
                if (proximityVec[i].size() > layer_no)
                    layer.push_back(&proximityVec[i][layer_no].center);
            }
        }
        if (layer.empty())
            break;

        int randomPick = rand() % layer.size();
        LEDs.push_back(*layer[randomPick]);
        layer.erase(layer.begin() + randomPick);
    }



    cout << endl << "RETURNED LEDS" << endl;
    for (int i=0; i<LEDs.size(); i++)
    {
        cout << " Point#: " << i << endl;
        cout << "  center: " << LEDs[i].x << " " << LEDs[i].y << endl;
    }


    // assign any mising points with old reprojected values
//    for (int i=0; i<NO_LEDS; i++)
//    {
//        // if we do not have any points in proximity of this LED, use the last reprojected image point
//        if (proximityVec[i].empty())
//        {
//            proximityVec[i].resize(1);
//            proximityVec[i][0].center = reprojImgPts[i];
//        }
//        leds.push_back(proximityVec[i][0].center);
        return 0;
}
