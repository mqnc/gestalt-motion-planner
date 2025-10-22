
#include "wraphtml.h"
#include "str.h"
#include "html/renderer.html.h"

string wrapHtml(const string& json){
	return str::replace(rendererHtml, "/*OBJECTS*/",
				string("let sceneData=") + json);
}
