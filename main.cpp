#include <bits/stdc++.h>
using namespace std;

/*
-------------------------------------------------------
 Legal Document Comparison using Suffix Array + LCP
 (File-paths hardcoded)
-------------------------------------------------------
*/

// ---------- Build Suffix Array ----------
vector<int> buildSuffixArray(const string &s) {
    int n = s.size();
    vector<int> sa(n), c(n), cnt(max(256, n), 0);

    for (int i = 0; i < n; ++i) cnt[s[i]]++;
    for (int i = 1; i < 256; ++i) cnt[i] += cnt[i - 1];
    for (int i = 0; i < n; ++i) sa[--cnt[s[i]]] = i;

    c[sa[0]] = 0;
    int classes = 1;
    for (int i = 1; i < n; ++i) {
        if (s[sa[i]] != s[sa[i - 1]]) classes++;
        c[sa[i]] = classes - 1;
    }

    vector<int> sa_tmp(n), c_tmp(n);
    for (int h = 0; (1 << h) < n; ++h) {
        for (int i = 0; i < n; ++i) {
            sa_tmp[i] = sa[i] - (1 << h);
            if (sa_tmp[i] < 0) sa_tmp[i] += n;
        }

        fill(cnt.begin(), cnt.begin() + classes, 0);
        for (int i = 0; i < n; ++i) cnt[c[sa_tmp[i]]]++;
        for (int i = 1; i < classes; ++i) cnt[i] += cnt[i - 1];
        for (int i = n - 1; i >= 0; --i)
            sa[--cnt[c[sa_tmp[i]]]] = sa_tmp[i];

        c_tmp[sa[0]] = 0;
        classes = 1;
        for (int i = 1; i < n; ++i) {
            pair<int, int> cur = {c[sa[i]], c[(sa[i] + (1 << h)) % n]};
            pair<int, int> prev = {c[sa[i - 1]], c[(sa[i - 1] + (1 << h)) % n]};
            if (cur != prev) ++classes;
            c_tmp[sa[i]] = classes - 1;
        }
        c.swap(c_tmp);
    }
    return sa;
}

// ---------- Build LCP Array ----------
vector<int> buildLCP(const string &s, const vector<int> &sa) {
    int n = s.size();
    vector<int> rank(n, 0), lcp(n - 1, 0);
    for (int i = 0; i < n; ++i) rank[sa[i]] = i;
    int k = 0;
    for (int i = 0; i < n; ++i) {
        if (rank[i] == n - 1) { k = 0; continue; }
        int j = sa[rank[i] + 1];
        while (i + k < n && j + k < n && s[i + k] == s[j + k]) ++k;
        lcp[rank[i]] = k;
        if (k) --k;
    }
    return lcp;
}

// ---------- Read File into String ----------
string readFile(const string &path) {
    ifstream file(path);
    if (!file) {
        cerr << "Error: Unable to open file -> " << path << "\n";
        exit(1);
    }
    stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// ---------- Main ----------
int main() {
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

    cout << "============================================\n";
    cout << " Legal Document Similarity using Suffix Array\n";
    cout << "============================================\n\n";

    // 🔹 HARD-CODE YOUR FILE PATHS HERE
    string path1 = "d:/Desktop/WSN(ADSA_Project)/doc1.txt";
    string path2 = "d:/Desktop/WSN(ADSA_Project)/doc2.txt";

    // Read both files
    string doc1 = readFile(path1);
    string doc2 = readFile(path2);

    // Combine with unique separator
    string combined = doc1 + char('$') + doc2 + char('#');
    int n = combined.size();

    // Assign doc IDs
    vector<int> docID(n);
    int curDoc = 1;
    for (int i = 0; i < n; ++i) {
        if (combined[i] == '$') curDoc = 2;
        docID[i] = curDoc;
    }

    // Build SA and LCP
    auto sa = buildSuffixArray(combined);
    auto lcp = buildLCP(combined, sa);

    // Find longest common substring between docs
    int maxLen = 0, pos = -1;
    for (int i = 1; i < n; ++i) {
        if (docID[sa[i]] != docID[sa[i - 1]] && lcp[i - 1] > maxLen) {
            maxLen = lcp[i - 1];
            pos = sa[i];
        }
    }

    cout << "Comparing:\n";
    cout << " → " << path1 << "\n";
    cout << " → " << path2 << "\n\n";

    cout << "--------------------------------------------\n";
    if (maxLen == 0) {
        cout << "No common clauses found.\n";
    } else {
        cout << "Longest common clause length: " << maxLen << " characters\n";
        cout << "--------------------------------------------\n";
        cout << combined.substr(pos, maxLen) << "\n";
    }
    cout << "--------------------------------------------\n";

    return 0;
}
