#!/usr/bin/env node

import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import {
  CallToolRequestSchema,
  ListToolsRequestSchema,
  TextContent
} from "@modelcontextprotocol/sdk/types.js";
import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";

// Get project root (parent of mcp-server directory)
const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const PROJECT_ROOT = path.resolve(__dirname, "..");

// Initialize the server
const server = new Server({
  name: "hover-combat-mcp",
  version: "1.0.0"
});

// Tool implementations
const tools = [
  {
    name: "read_file",
    description: "Read a file from the HoverCombat project. Path should be relative to project root.",
    inputSchema: {
      type: "object",
      properties: {
        path: {
          type: "string",
          description: "Relative path from project root (e.g., 'Assets/Scripts/HoverController.cs')"
        }
      },
      required: ["path"]
    }
  },
  {
    name: "list_files",
    description: "List files in a directory",
    inputSchema: {
      type: "object",
      properties: {
        path: {
          type: "string",
          description: "Directory path relative to project root (default: project root)"
        },
        pattern: {
          type: "string",
          description: "Optional file extension filter (e.g., '.cs', '.json')"
        }
      }
    }
  },
  {
    name: "search_files",
    description: "Search for files or content in the project",
    inputSchema: {
      type: "object",
      properties: {
        query: {
          type: "string",
          description: "Search query - filename to find or text to search for"
        },
        searchContent: {
          type: "boolean",
          description: "If true, search file contents; if false, search filenames"
        }
      },
      required: ["query"]
    }
  },
  {
    name: "get_project_info",
    description: "Get information about the HoverCombat project structure and key files",
    inputSchema: {
      type: "object",
      properties: {}
    }
  }
];

// Register tools
server.setRequestHandler(ListToolsRequestSchema, async () => ({
  tools: tools
}));

// Register tool call handler
server.setRequestHandler(CallToolRequestSchema, async (request) => {
  const { name, arguments: args } = request;

  try {
    switch (name) {
      case "read_file": {
        const filePath = path.join(PROJECT_ROOT, args.path);
        // Security: ensure path is within project root
        if (!filePath.startsWith(PROJECT_ROOT)) {
          return {
            content: [{ type: "text", text: "Error: Access denied - path outside project root" }],
            isError: true
          };
        }
        
        if (!fs.existsSync(filePath)) {
          return {
            content: [{ type: "text", text: `Error: File not found: ${args.path}` }],
            isError: true
          };
        }

        const content = fs.readFileSync(filePath, "utf-8");
        return {
          content: [{ type: "text", text: content }]
        };
      }

      case "list_files": {
        const dirPath = path.join(PROJECT_ROOT, args.path || "");
        
        if (!fs.existsSync(dirPath)) {
          return {
            content: [{ type: "text", text: `Error: Directory not found: ${args.path}` }],
            isError: true
          };
        }

        const entries = fs.readdirSync(dirPath, { withFileTypes: true });
        let files = entries.map(e => ({
          name: e.name,
          isDirectory: e.isDirectory(),
          path: path.relative(PROJECT_ROOT, path.join(dirPath, e.name))
        }));

        if (args.pattern) {
          files = files.filter(f => !f.isDirectory && f.name.endsWith(args.pattern));
        }

        return {
          content: [{ 
            type: "text", 
            text: files.map(f => `${f.isDirectory ? "[DIR] " : ""}${f.path}`).join("\n") 
          }]
        };
      }

      case "search_files": {
        const results = [];
        
        function searchDir(dir, maxDepth = 10) {
          if (maxDepth === 0) return;
          
          try {
            const entries = fs.readdirSync(dir, { withFileTypes: true });
            
            for (const entry of entries) {
              const fullPath = path.join(dir, entry.name);
              const relativePath = path.relative(PROJECT_ROOT, fullPath);
              
              if (entry.isDirectory()) {
                // Skip common directories
                if ([".git", "node_modules", "Library", "Temp", "bin", "obj"].includes(entry.name)) {
                  continue;
                }
                searchDir(fullPath, maxDepth - 1);
              } else {
                // Search filename
                if (!args.searchContent && entry.name.toLowerCase().includes(args.query.toLowerCase())) {
                  results.push(relativePath);
                }
                
                // Search content (only for text files)
                if (args.searchContent && entry.name.match(/\.(cs|txt|json|md|yaml|yml|xml)$/i)) {
                  try {
                    const content = fs.readFileSync(fullPath, "utf-8");
                    if (content.toLowerCase().includes(args.query.toLowerCase())) {
                      results.push(relativePath);
                    }
                  } catch {
                    // Skip files that can't be read
                  }
                }
              }
            }
          } catch {
            // Skip directories we can't read
          }
        }

        searchDir(PROJECT_ROOT);
        
        return {
          content: [{ 
            type: "text", 
            text: results.length > 0 ? results.join("\n") : "No files found" 
          }]
        };
      }

      case "get_project_info": {
        const info = {
          projectRoot: PROJECT_ROOT,
          files: {
            csproj: fs.readdirSync(PROJECT_ROOT).filter(f => f.endsWith(".csproj")),
            sln: fs.readdirSync(PROJECT_ROOT).filter(f => f.endsWith(".sln")),
            assets: fs.existsSync(path.join(PROJECT_ROOT, "Assets")),
            scriptsDir: fs.existsSync(path.join(PROJECT_ROOT, "Assets", "Scripts"))
          }
        };
        
        // Get C# script files
        const scriptsPath = path.join(PROJECT_ROOT, "Assets", "Scripts");
        if (fs.existsSync(scriptsPath)) {
          const scripts = fs.readdirSync(scriptsPath).filter(f => f.endsWith(".cs"));
          info.scripts = scripts;
        }

        return {
          content: [{ 
            type: "text", 
            text: JSON.stringify(info, null, 2) 
          }]
        };
      }

      default:
        return {
          content: [{ type: "text", text: `Unknown tool: ${name}` }],
          isError: true
        };
    }
  } catch (error) {
    return {
      content: [{ type: "text", text: `Error: ${error.message}` }],
      isError: true
    };
  }
});

// Start server
const transport = new StdioServerTransport();
await server.connect(transport);
