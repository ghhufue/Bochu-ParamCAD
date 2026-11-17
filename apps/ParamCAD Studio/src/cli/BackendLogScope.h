#pragma once
#include <cstdio>
#include <iostream>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif
namespace pcad {
// OCCT transfer diagnostics use C stdout. Preserve them on stderr while the
// CLI's stdout remains one JSON result. Scope only synchronous backend work.
class BackendLogScope {
    int saved=-1;
public:
    BackendLogScope(){std::cout.flush();std::fflush(stdout);
#ifdef _WIN32
        saved=_dup(_fileno(stdout));if(saved>=0)_dup2(_fileno(stderr),_fileno(stdout));
#else
        saved=dup(fileno(stdout));if(saved>=0)dup2(fileno(stderr),fileno(stdout));
#endif
    }
    ~BackendLogScope(){std::cout.flush();std::fflush(stdout);if(saved<0)return;
#ifdef _WIN32
        _dup2(saved,_fileno(stdout));_close(saved);
#else
        dup2(saved,fileno(stdout));close(saved);
#endif
    }
};
}
