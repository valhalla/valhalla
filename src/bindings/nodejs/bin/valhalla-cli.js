#!/usr/bin/env node

const { spawn } = require('child_process');
const path = require('path');
const fs = require('fs').promises;
const fsSync = require('fs');
const { getBinaryDir } = require('../lib/binary-path');

const binaryDir = getBinaryDir(path.join(__dirname, '..'));

// Enable debug logging if VALHALLA_DEBUG env var is set
const DEBUG = process.env.VALHALLA_DEBUG === '1' || process.env.VALHALLA_DEBUG === 'true';

function debugLog(...args) {
    if (DEBUG) {
        console.error('[DEBUG]', ...args);
    }
}

async function fileExists(filePath) {
    try {
        await fs.access(filePath);
        return true;
    } catch {
        return false;
    }
}

async function ensureExecutable(filePath) {
    try {
        const stats = await fs.stat(filePath);
        debugLog(`File stats for ${path.basename(filePath)}:`, {
            mode: stats.mode.toString(8),
            isExecutable: !!(stats.mode & fsSync.constants.S_IXUSR)
        });
        
        // Check if file has execute permission
        const hasExecutePerm = !!(stats.mode & fsSync.constants.S_IXUSR);
        
        if (!hasExecutePerm) {
            debugLog(`Adding execute permission to ${filePath}`);
            // Add execute permission for user, group, and others
            await fs.chmod(filePath, stats.mode | 0o111);
            debugLog(`Execute permission added successfully`);
        }
    } catch (err) {
        debugLog(`Error ensuring executable for ${filePath}:`, err.message);
        throw err;
    }
}

async function getAvailableCommands() {
    try {
        const files = await fs.readdir(binaryDir);
        const commands = [];
        for (const file of files) {
            const filePath = path.join(binaryDir, file);
            
            try {
                const stats = await fs.stat(filePath);
                
                if (!stats.isFile()) {
                    continue;
                }
                
                if (file.endsWith('.node')) {
                    continue;
                }
                
                // Check if file is executable (has execute permission)
                await fs.access(filePath, fsSync.constants.X_OK);
                commands.push(file);
            } catch {
                continue;
            }
        }
        
        return commands;
    } catch {
        return [];
    }
}

async function printHelp() {
    const packageJson = require('../package.json');
    const commands = await getAvailableCommands();
    
    // Get max command length for alignment
    let maxLen = 0;
    for (const cmd of commands) {
        if (cmd.length > maxLen) {
            maxLen = cmd.length;
        }
    }
    
    console.log(`Valhalla Node.js package v${packageJson.version}\n`);
    console.log('valhalla CLI provides access to Valhalla C++ executables. Arguments are passed through as-is.\n');
    console.log('Usage: valhalla <command> [args...]\n');
    console.log('Options:');
    console.log('  --help, -h     Show this help message');
    console.log('  --version, -v  Show package version');
    console.log('  print-bin-path Print the absolute path to the binaries directory\n');
    
    if (commands.length > 0) {
        console.log(`Available commands (in ${binaryDir}):`);
        for (const cmd of commands) {
            // Show shorthand version without valhalla_ prefix for convenience
            const displayName = cmd.startsWith('valhalla_') ? cmd.substring(9) : cmd;
            const padding = ' '.repeat(Math.max(0, maxLen - cmd.length));
            console.log(`  ${cmd}${padding}  - Run 'valhalla ${displayName} --help' for command help`);
        }
    } else {
        console.log('No binaries found. Make sure the package is properly installed.');
    }
    
    console.log('\nNote: You can use commands with or without the "valhalla_" prefix.');
    console.log('\nExamples:');
    console.log('  valhalla build_tiles -c config.json data.osm.pbf');
    console.log('  valhalla run_route -c config.json -j \'{"locations":[...]}\'');
    console.log('  valhalla valhalla_service -c config.json  # Full name also works');
}

async function runCommand(command, args) {
    debugLog(`Binary directory: ${binaryDir}`);
    debugLog(`Looking for command: ${command}`);
    
    // Try to find the binary with the given name first
    let binaryPath = path.join(binaryDir, command);
    
    // Check if it exists
    let exists = await fileExists(binaryPath);
    debugLog(`Checking ${binaryPath}: ${exists ? 'found' : 'not found'}`);
    
    // If not found and doesn't start with 'valhalla_', try with 'valhalla_' prefix
    if (!exists && !command.startsWith('valhalla_')) {
        const prefixedCommand = `valhalla_${command}`;
        binaryPath = path.join(binaryDir, prefixedCommand);
        exists = await fileExists(binaryPath);
        debugLog(`Checking ${binaryPath}: ${exists ? 'found' : 'not found'}`);
    }
    
    if (!exists) {
        console.error(`Error: Command '${command}' not found in ${binaryDir}`);
        console.error(`Tried: ${command}${!command.startsWith('valhalla_') ? ` and valhalla_${command}` : ''}`);
        console.error(`Run 'valhalla --help' to see available commands.`);
        debugLog(`Binary directory contents:`, await fs.readdir(binaryDir).catch(() => []));
        process.exit(1);
    }
    
    // Ensure the binary is executable
    try {
        await ensureExecutable(binaryPath);
    } catch (err) {
        console.error(`Error ensuring executable permissions for ${binaryPath}:`, err.message);
        console.error(`Try running: chmod +x ${binaryPath}`);
        process.exit(1);
    }
    
    debugLog(`Spawning: ${binaryPath}`, args);
    
    // Spawn the binary
    const proc = spawn(binaryPath, args, {
        stdio: 'inherit', // Pass through stdin, stdout, stderr
        env: process.env
    });
    
    proc.on('error', (err) => {
        console.error(`Error running ${command}:`, err.message);
        debugLog(`Full error:`, err);
        process.exit(1);
    });
    
    proc.on('close', (code) => {
        debugLog(`Process exited with code: ${code}`);
        process.exit(code || 0);
    });
}

async function main() {
    const args = process.argv.slice(2);
    
    // Handle no arguments
    if (args.length === 0) {
        await printHelp();
        process.exit(0);
    }
    
    const command = args[0];
    const commandArgs = args.slice(1);
    
    if (command === '--help' || command === '-h') {
        await printHelp();
        process.exit(0);
    }
    
    if (command === '--version' || command === '-v') {
        const packageJson = require('../package.json');
        console.log(packageJson.version);
        process.exit(0);
    }
    
    await runCommand(command, commandArgs);
}

main().catch(err => {
    console.error('Error:', err.message);
    process.exit(1);
});

