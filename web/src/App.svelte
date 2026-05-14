<script>
  import { onMount } from 'svelte';

  const linearSpeed = 60;
  const angularSpeed = 1.5;

  let environment = {
    width: 1,
    height: 1,
    resolution: 1,
    robotRadius: 1,
    mapImage: '/api/map-image'
  };
  let obstacles = {
    circles: [],
    rectangles: []
  };
  let station = null;
  let waste = [];
  let scan = [];
  let game = null;

  let robot = {
    x: 0,
    y: 0,
    theta: 0,
    linear: 0,
    angular: 0,
    collision: false
  };

  let connected = false;
  let activeCommand = 'stop';
  let pressedKeys = new Set();
  let commandTimer;
  let controlStopTimer;
  let stateTimer;
  let sceneLoaded = false;
  let activePointerId = null;

  $: viewBox = `0 0 ${environment.width} ${environment.height}`;
  $: robotSize = Math.max(environment.robotRadius * 2, 1);
  $: headingX = robot.x + Math.cos(robot.theta) * environment.robotRadius * 1.7;
  $: headingY = robot.y + Math.sin(robot.theta) * environment.robotRadius * 1.7;
  $: hasSceneGeometry =
    obstacles.circles.length > 0 ||
    obstacles.rectangles.length > 0 ||
    station ||
    waste.length > 0;
  $: showEmptyGame = !game && (!sceneLoaded || !hasSceneGeometry);

  async function loadScene() {
    const response = await fetch('/api/scene');
    const scene = await response.json();
    environment = scene.environment;
    obstacles = scene.obstacles;
    station = scene.station;
    waste = scene.waste ?? [];
    sceneLoaded = true;
  }

  async function pollState() {
    try {
      const response = await fetch('/api/state', { cache: 'no-store' });
      const data = await response.json();
      robot = data.robot;
      scan = data.scan ?? [];
      waste = data.waste ?? waste;
      game = data.game;
      connected = true;
    } catch {
      connected = false;
    }
  }

  function velocityForCommand(command) {
    switch (command) {
      case 'forward':
        return { linear: linearSpeed, angular: 0 };
      case 'back':
        return { linear: -linearSpeed, angular: 0 };
      case 'left':
        return { linear: 0, angular: angularSpeed };
      case 'right':
        return { linear: 0, angular: -angularSpeed };
      default:
        return { linear: 0, angular: 0 };
    }
  }

  async function postVelocity(velocity) {
    await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(velocity)
    }).then((response) => {
      if (!response.ok) connected = false;
    }).catch(() => {
      connected = false;
    });
  }

  async function sendCommand(command = activeCommand) {
    await postVelocity(velocityForCommand(command));
  }

  async function restartGame() {
    pressedKeys.clear();
    activeCommand = 'stop';
    await postVelocity({ linear: 0, angular: 0 });

    const response = await fetch('/api/game/restart', { method: 'POST' });
    if (!response.ok) {
      connected = false;
      return;
    }

    await loadScene();
    await pollState();
  }

  function setCommand(command) {
    if (activeCommand === command) return;
    activeCommand = command;
    sendCommand(command);
  }

  function stop() {
    setCommand('stop');
  }

  function handleControlDown(event, command) {
    event.preventDefault();
    clearTimeout(controlStopTimer);
    activePointerId = event.pointerId;
    event.currentTarget.setPointerCapture?.(activePointerId);
    setCommand(command);
  }

  function handleControlUp(event) {
    if (event && activePointerId !== null && event.pointerId !== activePointerId) return;
    event?.currentTarget?.releasePointerCapture?.(activePointerId);
    activePointerId = null;
    clearTimeout(controlStopTimer);
    stop();
  }

  function updateKeyboardCommand() {
    if (pressedKeys.has('w') || pressedKeys.has('arrowup')) setCommand('forward');
    else if (pressedKeys.has('s') || pressedKeys.has('arrowdown')) setCommand('back');
    else if (pressedKeys.has('a') || pressedKeys.has('arrowleft')) setCommand('left');
    else if (pressedKeys.has('d') || pressedKeys.has('arrowright')) setCommand('right');
    else stop();
  }

  function handleKeyDown(event) {
    const key = event.key.toLowerCase();
    if (!['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright', ' '].includes(key)) return;
    event.preventDefault();
    if (key === ' ') {
      pressedKeys.clear();
      stop();
      return;
    }
    pressedKeys.add(key);
    updateKeyboardCommand();
  }

  function handleKeyUp(event) {
    const key = event.key.toLowerCase();
    pressedKeys.delete(key);
    updateKeyboardCommand();
  }

  onMount(() => {
    loadScene().catch(() => {
      connected = false;
    });
    pollState();
    stateTimer = setInterval(pollState, 100);
    commandTimer = setInterval(() => sendCommand(), 50);
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    return () => {
      clearInterval(stateTimer);
      clearInterval(commandTimer);
      clearTimeout(controlStopTimer);
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      stop();
    };
  });
</script>

<main>
  <section class="viewport">
    <div class="map-shell">
      {#if showEmptyGame}
        <div class="empty-game" role="status" aria-label="Keep Clean is not playing">
          <div class="empty-arena">
            <span class="empty-station"></span>
            <span class="empty-robot"></span>
            <span class="empty-waste waste-one"></span>
            <span class="empty-waste waste-two"></span>
            <span class="empty-waste waste-three"></span>
          </div>
          <div class="empty-copy">
            <p>Keep Clean</p>
            <h2>Ready for a new run</h2>
            <button type="button" on:click={restartGame}>Start Game</button>
          </div>
        </div>
      {:else}
        <svg viewBox={viewBox} role="img" aria-label="Robot map">
          <image href={environment.mapImage} x="0" y="0" width={environment.width} height={environment.height} preserveAspectRatio="none" transform={`translate(0 ${environment.height}) scale(1 -1)`} />

          {#each obstacles.rectangles as obstacle}
            <rect class="obstacle" x={obstacle.x} y={environment.height - obstacle.y - obstacle.height} width={obstacle.width} height={obstacle.height} />
          {/each}

          {#each obstacles.circles as obstacle}
            <circle class="obstacle" cx={obstacle.x} cy={environment.height - obstacle.y} r={obstacle.radius} />
          {/each}

          {#if station}
            <circle class="station" cx={station.x} cy={environment.height - station.y} r={station.radius} />
          {/if}

          {#each waste as item}
            <circle class="waste" cx={item.x} cy={environment.height - item.y} r={item.radius} />
          {/each}

          <g class="scan">
            {#each scan as hit}
              <line x1={robot.x} y1={environment.height - robot.y} x2={hit.x} y2={environment.height - hit.y} />
              <circle cx={hit.x} cy={environment.height - hit.y} r="0.28" />
            {/each}
          </g>

          <circle class:collision={robot.collision} class="robot-body" cx={robot.x} cy={environment.height - robot.y} r={robotSize / 2} />
          <line class="robot-heading" x1={robot.x} y1={environment.height - robot.y} x2={headingX} y2={environment.height - headingY} />
        </svg>
      {/if}
    </div>
  </section>

  <aside class="panel">
    <div class="game-menu">
      <p>Game Mode</p>
      <h1>Keep Clean</h1>
      <div class="menu-actions">
        <button class="menu-action active" type="button" on:click={restartGame}>Play</button>
        <button class="menu-action" type="button" on:click={restartGame}>Restart</button>
      </div>
    </div>

    <div class="status">
      <span class:online={connected}></span>
      {connected ? 'Connected' : 'Disconnected'}
    </div>

    <div class="readout">
      <div>
        <strong>{robot.x.toFixed(1)}</strong>
        <span>x</span>
      </div>
      <div>
        <strong>{robot.y.toFixed(1)}</strong>
        <span>y</span>
      </div>
      <div>
        <strong>{robot.theta.toFixed(2)}</strong>
        <span>theta</span>
      </div>
    </div>

    {#if game}
      <div class="game">
        <div>
          <strong>{game.score || 0}</strong>
          <span>score</span>
        </div>
        <div>
          <strong>{game.capacity || 0}/{game.maxCapacity || 0}</strong>
          <span>capacity</span>
        </div>
        <div>
          <strong>{game.currentWave || 0}/{game.totalWaves || 0}</strong>
          <span>wave</span>
        </div>
      </div>

      <div class="game">
        <div>
          <strong>{game.collectedInWave || 0}/{game.requiredPerWave || 0}</strong>
          <span>collected</span>
        </div>
        <div>
          <strong>{Math.max(0, (game.waveTimeLimitSeconds || 0) - (game.waveElapsedSeconds || 0)).toFixed(1)}</strong>
          <span>time</span>
        </div>
      </div>

      {#if game.finished}
        <div class:success={game.success} class="result">
          {game.success ? 'Success' : 'Failed'}: {game.endReason}
        </div>
      {/if}

      <button class="restart" on:click={restartGame}>Restart</button>
    {/if}

    <div class="controls" role="group" aria-label="Robot movement controls" on:mouseleave={stop} on:pointercancel={stop}>
      <button type="button" aria-label="Move forward" class:active={activeCommand === 'forward'} on:pointerdown={(event) => handleControlDown(event, 'forward')} on:pointerup={handleControlUp} on:pointercancel={handleControlUp}>&uarr;</button>
      <button type="button" aria-label="Turn left" class:active={activeCommand === 'left'} on:pointerdown={(event) => handleControlDown(event, 'left')} on:pointerup={handleControlUp} on:pointercancel={handleControlUp}>&larr;</button>
      <button type="button" aria-label="Move backward" class:active={activeCommand === 'back'} on:pointerdown={(event) => handleControlDown(event, 'back')} on:pointerup={handleControlUp} on:pointercancel={handleControlUp}>&darr;</button>
      <button type="button" aria-label="Turn right" class:active={activeCommand === 'right'} on:pointerdown={(event) => handleControlDown(event, 'right')} on:pointerup={handleControlUp} on:pointercancel={handleControlUp}>&rarr;</button>
    </div>
  </aside>
</main>
