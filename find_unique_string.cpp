#include <set>
#include <string>
#include <iostream>

using namespace std;

int main()
{
    set<string> str;
    string s;
    getline(cin, s);
    while (s != "q")
    {
        getline(cin, s);
        str.insert(s);
    }

    cout << "result: " << endl;
    for (string s : str)
    {
        cout << s << endl;
    }
}
