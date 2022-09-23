#include <iostream>
#include <fstream>
#include <string>
#include <string.h>
#include <sstream>
#include <vector>

using namespace std;

int main(){
    int i = 0;
    fstream newfile;
    string word;
    double num;
    ofstream joint;
    joint.open("square_joint_val.txt");
    newfile.open("square.txt",ios::in);
    if(newfile.is_open()){
        string tp;
        while(getline(newfile, tp)){
            stringstream ss(tp);
            int count = 0;
            vector<double> v;
            while(ss >> word){
                if(word.compare("position:")==0){
                    
                    while(ss>>word){
                        count++;
                        if(count == 1){
                            word = word.substr(1,word.length()-1);
                        }
                        if(stringstream(word) >> num){
                        cout << num << " ";
                        v.push_back(num);
                        }
                    }
                    for(int k = 0;k < 6;k++){
                        if(k==5){
                            joint << v[k] << endl;
                        }
                        else{
                            joint << v[k] << " ";
                        }
                    }
                }
            }
            // cout << tp << "\n";
            i++;
        }
        newfile.close();
    }
}
