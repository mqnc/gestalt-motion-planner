
#include "peglib.h"
#include <assert.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <regex>
#include <typeinfo>
#include <typeindex>

using namespace std;
using namespace std::string_literals;
using namespace peg;

// load string from file
inline string load(const string& filename, bool binary = false) {
	ifstream ifs(filename, binary ? ios::in | ios::binary : ios::in);
	if (!ifs) {
		throw runtime_error(filename + " not found");
	}
	return string(
		(istreambuf_iterator<char>(ifs)),
		(istreambuf_iterator<char>()));
}

// save string to file
inline void save(const string& filename, const string& content) {
	ofstream ofs(filename);
	if (!ofs) {
		throw runtime_error(filename + " could not be written");
	}
	ofs << content;
}

inline void replaceInplace(
	std::string& str,
	const std::string& search,
	const std::string& repl
) {
	size_t start_pos = 0;
	while ((start_pos = str.find(search, start_pos)) != std::string::npos) {
		str.replace(start_pos, search.length(), repl);
		start_pos += repl.length(); // Handles case where 'repl' is a substring of 'search'
	}
}

inline std::string replace(
	std::string str,
	const std::string& search,
	const std::string& repl
) {
	replaceInplace(str, search, repl);
	return str;
}

int main(int argc, char* argv[]) {

	if (argc < 6) {
		cout << "usage:\n" << argv[0]
			<< " GRAMMAR_FILE OUTPUT_FILE CLASS_NAME INPUT_FILES\n";
		return EXIT_FAILURE;
	}

	try {
		string grammar = load(argv[1]);
		string outputFile = argv[2];
		string jsonOutputFile = argv[3];
		string className = argv[4];
		vector<string> inputFiles;
		for (size_t i = 5; i < argc; i++) {
			inputFiles.push_back(argv[i]);
		}

		grammar = replace(grammar, "{CLASS}", "\""s + className + "\""s);

		parser parser;

		parser.log = [](size_t line, size_t col, const string& msg) {
			cout << "grammar error (" << line << ":" << col << "): " << msg << "\n";
		};

		parser.load_grammar(grammar);
		if (static_cast<bool>(parser) == false) {
			return EXIT_FAILURE;
		}

		for (const auto& rule : parser.get_rule_names()) {
			parser[rule.c_str()] = [](const SemanticValues& vs) {
				if (vs.tokens.size() == 0) {
					stringstream ss;
					for (const auto& v : vs) {
						ss << any_cast<string>(v);
					}
					return string(ss.str());
				}
				else {
					return string(vs.token_to_string());
				}
			};
		}

		map<string, string> privates;
		map<string, string> publics;
		map<string, string> jsonDispatches;

		struct Argument {
			string type;
			string identifier;
			string dflt;
		};

		parser["Arg"] = [&](const SemanticValues& vs) {
			return Argument{
				string{any_cast<string>(vs[0])},
				string{any_cast<string>(vs[1])},
				string{vs.size() > 2 ? (" " + any_cast<string>(vs[2])) : ""}
			};
		};

		parser["Args"] = [&](const SemanticValues& vs) {
			vector<Argument> currentArgs;
			for (const auto& v : vs) {
				currentArgs.push_back(any_cast<Argument>(v));
			}
			return currentArgs;
		};

		parser["MemberDeclarationHeader"] = [&](const SemanticValues& vs) {
			string qualifier{ vs.tokens[0] };
			string returnType, fullName;
			if (vs.tokens.size() == 2) {
				returnType = "void";
				fullName = vs.tokens[1];
			}
			else {
				returnType = replace(string(vs.tokens[1]), " ", "");
				fullName = vs.tokens[2];
			}

			string name = replace(fullName, className + "::", "");

			stringstream body;
			vector<Argument> args;

			for (const auto& v : vs) {
				if (v.type() == typeid(string)) {
					body << any_cast<string>(v);
				}
				else if (v.type() == typeid(vector<Argument>)) {
					args = any_cast<vector<Argument>>(v);
					body << "(";
					string sep = "";
					for (const auto& arg : args) {
						body << sep << "\n\t" << arg.type << " " << arg.identifier << arg.dflt;
						sep = ",";
					}
					body << ");";
				}
			}

			if (qualifier.find("private") != string::npos) {
				privates[fullName] = body.str();
			}
			else if (qualifier.find("public") != string::npos) {
				if (args.size() > 0 && name != className) {
					// generate api for C++20 designated initializers
					body << "\n\n#ifndef PYBIND";
					body << "\npublic: struct " << name << "_params{";
					for (const auto& arg : args) {
						body << "\n\t" << arg.type << " " << arg.identifier << arg.dflt << ";";
					}
					body << " };\n\n";
					for (const auto& v : vs) {
						if (v.type() == typeid(string)) {
							body << any_cast<string>(v);
						}
					}
					body << "(" << name << "_params param_object){\n"
						<< "\treturn " << name << "(\n";
					string sep = "";
					for (const auto& arg : args) {
						body << sep << "\t\tparam_object." << arg.identifier;
						sep = ",\n";
					}
					body << ");\n}";
					body << "\n#endif\n";
				}
				publics[fullName] = body.str();

				if (name != className && name[0] != '~') {
					// generate json dispatch

					stringstream ss;
					ss << "if (method == \"" << name << "\") {\n\t";
					if (returnType != "void") {
						ss << "result = ";
					}
					ss << "self." << name << "(";
					string sep = "\n";
					for (const auto& arg : args) {
						if (arg.dflt == "") {
							ss << sep << "\t\tparams.at(\"" << arg.identifier << "\").get<"
								<< replace(arg.type, "&", "") << ">()";
						}
						else {
							string dflt = arg.dflt;
							while (
								dflt[0] == ' ' ||
								dflt[0] == '\t' ||
								dflt[0] == '\n' ||
								dflt[0] == '\r' ||
								dflt[0] == '='
								) {
								dflt.erase(0, 1);
							}
							if (dflt[0] == '{') {
								dflt = replace(
									replace(
										arg.type, "&", ""
									), "const ", "")
									+ dflt;
							}
							ss << sep << "\t\tparams.value(\"" << arg.identifier << "\", "
								<< dflt << ")";
						}
						sep = ",\n";
					}
					ss << ");\n}";
					jsonDispatches[fullName] = ss.str();
				}
			}
			else {
				throw runtime_error(
					string(fullName) + "don't know if public or private");
			}

			return string("");
		};

		parser["Orphan"] = [](const SemanticValues& vs) {
			throw runtime_error(
				string("something's wrong with the declaration of ") +
				vs.token_to_string()
			);
		};

		parser.enable_packrat_parsing();

		for (const auto& file : inputFiles) {
			parser.parse(load(file));
		}

		{
			stringstream ss;

			ss << "\n// This file is automatically generated "
				"by the coding assistants. Don't edit it manually.\n"
				"// The member declarations are generated "
				"from the definitions in the cpp files.\n\n";

			for (const auto& [k, v] : privates) {
				ss << v << "\n\n";
			}

			for (const auto& [k, v] : publics) {
				ss << v << "\n\n";
			}

			string output = ss.str();
			bool  create = true;
			try {
				string old = load(outputFile);
				if (output == old) {
					cout << outputFile << " would not change, not overwriting\n";
					create = false;
				}
			}
			catch (runtime_error& e) {}
			if (create) {
				save(outputFile, output);
			}
		}

		{
			stringstream ss;

			ss << "\n// This file is automatically generated "
				"by the coding assistants. Don't edit it manually.\n"
				"// The dispatch branches are extracted "
				"from the definitions in the cpp files.\n\n";

			string sep = "";
			for (const auto& [k, v] : jsonDispatches) {
				ss << sep << v;
				sep = "\n\nelse ";
			}

			string output = ss.str();
			bool  create = true;
			try {
				string old = load(jsonOutputFile);
				if (output == old) {
					cout << jsonOutputFile << " would not change, not overwriting\n";
					create = false;
				}
			}
			catch (runtime_error& e) {}
			if (create) {
				save(jsonOutputFile, output);
			}
		}

		return EXIT_SUCCESS;
	}
	catch (runtime_error& e) {
		cout << e.what() << "\n";
		return EXIT_FAILURE;
	}
}