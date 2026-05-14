import { svelte } from '@sveltejs/vite-plugin-svelte';
import { defineConfig } from 'vite';

export default defineConfig({
  plugins: [svelte()],
  server: {
    port: 5173,
    proxy: {
      '/api': {
        target: 'http://localhost:8080',
        configure(proxy) {
          proxy.on('error', (_error, _request, response) => {
            if (!response.headersSent) {
              response.writeHead(503, { 'Content-Type': 'application/json' });
            }
            response.end(JSON.stringify({ ok: false, error: 'backend unavailable' }));
          });
        }
      }
    }
  }
});
