#include "component/context_adaptation/ContextAdaptation.hpp"

int main(int argc, char **argv) {
    ContextAdaptation context_adaptation(argc, argv, "Context Adaptation");
    return context_adaptation.run();
}