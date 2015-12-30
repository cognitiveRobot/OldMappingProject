/* 
 * File:   generalFunctions.h
 * Author: Jean-Nicolas
 *
 * Created on 16 juin 2011, 12:30
 */
#include <vector>
#include "PointAndSurface.H"

using namespace std;


template <class T> 
vector<T> cloneSuppression(const vector<T> & v) {
    bool isUseful[v.size()];
    for (unsigned i = 0; i < v.size(); i++) {
        isUseful[i] = true;
    }

    for (unsigned i = 0; i < v.size(); i++) {
        if (isUseful[i]) {
            for (unsigned j = i + 1; j < v.size(); j++) {
                if (v[i] == v[j]) {
                    isUseful[j] = false;
                }
            }
        }
    }

    vector<T> result;
    for (unsigned i = 0; i < v.size(); i++) {
        if (isUseful[i]) {
            result.push_back(v[i]);
        }
    }
    return result;
}


template <class T> 
vector<T> intersection(const vector<T> & v1, const vector<T> & v2) {
    vector<T> result;
    
    for(unsigned i=0;i<v1.size();i++){
        for(unsigned j=0;j<v2.size();j++){
            if(v1[i]==v2[j]){
                result.push_back(v1[i]);
            }
        }
    }
    
    result = cloneSuppression(result);
    
    return result;
}


double vectorsAngleRad(const PointXY & v1, const PointXY & v2);
double vectorsAngleDeg(const PointXY & v1, const PointXY & v2);