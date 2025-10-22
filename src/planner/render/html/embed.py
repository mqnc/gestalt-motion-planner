
import os
import re
from base64 import b64encode

os.chdir(os.path.dirname(os.path.realpath(__file__)))

with open('index.html', 'r') as file:
	html = file.read()

def embedScript(linked):
	file_name = linked.group(1)
	with open(file_name, 'r') as file:
		js = file.read()
	return "<script>\n" + js + "\n</script>\n"

def embedImage(linked):
	file_name = linked.group(0)
	with open(file_name, 'rb') as f:
		b64 = str(b64encode(f.read()).decode())
		return "data:image/jpg;base64," + b64

html = re.sub('<script src="(.*)"></script>', embedScript, html)
html = re.sub('environment/..\.jpg', embedImage, html)



html_h = '''#pragma once
inline auto rendererHtml = R"tabs_over_spaces(
''' + html + ''')tabs_over_spaces";
'''

with open('renderer.html.h', 'w') as file:
	file.write(html_h)
