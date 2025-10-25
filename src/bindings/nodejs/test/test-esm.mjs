import valhalla from '@valhallajs/valhallajs';

async function main() {
  try {
    console.log('[SUCCESS] Valhalla package loaded successfully (ESM)!');
    console.log('[INFO] VALHALLA_VERSION:', valhalla.VALHALLA_VERSION);
    console.log('[INFO] Available exports:', Object.keys(valhalla));
    
    // Basic functionality test
    if (typeof valhalla.VALHALLA_VERSION === 'string' && valhalla.VALHALLA_VERSION.length > 0) {
      console.log('[SUCCESS] Version check passed');
    } else {
      console.error('[ERROR] Invalid version');
      process.exit(1);
    }
    
    if (typeof valhalla.Actor === 'function') {
      console.log('[SUCCESS] Actor class is exported');
      
      // Test Actor with config object
      try {
        const config = await valhalla.getConfig({ tileDir: './test_tiles' });
        const actor = new valhalla.Actor(config);
        
        if (actor && actor.actor) {
          console.log('[SUCCESS] Actor created with config object');
        } else {
          console.error('[ERROR] Actor instance invalid');
          process.exit(1);
        }
      } catch (error) {
        // It's okay if Actor creation fails due to missing tiles,
        // we just want to verify the API works
        if (error.message.includes('tiles') || error.message.includes('tile_dir')) {
          console.log('[SUCCESS] Actor accepts config object (tiles not available for full test)');
        } else {
          console.error('[ERROR] Actor creation failed:', error.message);
          process.exit(1);
        }
      }
      
      // Check fromConfigFile static method
      if (typeof valhalla.Actor.fromConfigFile === 'function') {
        console.log('[SUCCESS] Actor.fromConfigFile static method is exported');
      } else {
        console.error('[ERROR] Actor.fromConfigFile not found');
        process.exit(1);
      }
    } else {
      console.error('[ERROR] Actor class not found');
      process.exit(1);
    }
    
    console.log('[SUCCESS] All ESM tests passed!');
    process.exit(0);
  } catch (error) {
    console.error('[ERROR] Failed to load Valhalla package (ESM):', error.message);
    console.error(error.stack);
    process.exit(1);
  }
}

main();
