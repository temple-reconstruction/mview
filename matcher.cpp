#include "mview.h"

namespace std {
	void default_delete<Matcher>::operator()(Matcher* matcher) const {
		matcher->~Matcher();
	}
}
