const platformMap = { "1": "L", "2": "M", "3": "R" };

const ledModes = ["rainbow", "solid", "flashing", "theater_chase", "sindist", "windupdown", "pulsing"];
const ledModeDisplayMap = {"rainbow" : "Rainbow", "solid" : "Solid", "flashing" : "Flashing", "theater_chase" : "Theater Chase", "sindist" : "Sinusoidal Distribution", "windupdown" : "Wind Up/Down", "pulsing" : "Pulsing"};

const motorModes = ["continuous", "sinusoidal", "stop"];
const motorModeDisplayMap = { "continuous": "Continuous", "sinusoidal": "Sinusoidal", "stop": "Stop" };

document.addEventListener("DOMContentLoaded", function () {

    // Populate LED and motor dropdowns
    const ledControls = document.querySelectorAll(".led-control");
    const motorControls = document.querySelectorAll(".motor-control");

    ledControls.forEach((control) => {
        ledModes.forEach((mode) => {
            const option = document.createElement("option");
            option.value = mode;
            option.textContent = ledModeDisplayMap[mode];
            control.appendChild(option);
        });
    });

    motorControls.forEach((control) => {
        motorModes.forEach((mode) => {
            const option = document.createElement("option");
            option.value = mode;
            option.textContent = motorModeDisplayMap[mode];
            control.appendChild(option);
        });
    });

    // Event listeners for LED dropdowns
    ledControls.forEach((control) => {
        control.addEventListener("change", function () {
            const mode = this.value; // Get selected mode
            const platform = this.dataset.platform; // Get platform ID
            sendControlCommand(mode, platform, "LED", ledModeDisplayMap[mode]);
        });
    });

    // Event listeners for Motor dropdowns
    motorControls.forEach((control) => {
        control.addEventListener("change", function () {
            const mode = this.value; // Get selected mode
            const platform = this.dataset.platform; // Get platform ID
            sendControlCommand(mode, platform, "Motor", motorModeDisplayMap[mode]);
        });
    });

    function sendControlCommand(mode, platform, type, modeText) {
        const esp32Url = `http://192.168.0.55/${type.toLowerCase()}_${platform}_${mode}`; // Construct ESP32 URL dynamically
        console.log(esp32Url);
        fetch(esp32Url)
            .then((response) => {
                if (response.ok) {
                    return response.text();
                }
                throw new Error("Failed to fetch");
            })
            .then((data) => {
                updateFeedback(`Platform ${platformMap[platform]}: ${type} ${modeText}`, false);
            })
            .catch(() => {
                updateFeedback(`Failed to set ${type} mode for Platform ${platform}`, true);
            });
    }

    function updateFeedback(message, isError) {
        const feedback = document.getElementById("feedback");
        feedback.textContent = message;
        feedback.className = `alert ${isError ? "alert-danger" : "alert-success"}`;
        feedback.style.opacity = 1;
        feedback.style.transform = "translateY(0)";

        setTimeout(() => {
            feedback.style.opacity = 0;
            feedback.style.transform = "translateY(-200px)";
        }, 2000); // 3 seconds display time
    }
});

////////////////////   Animation   ////////////////////

// Initialize Three.js scene
const scene = new THREE.Scene();
const canvas = document.getElementById('shelf-visualizer');
const renderer = new THREE.WebGLRenderer({ canvas });

renderer.setSize(canvas.clientWidth, canvas.clientHeight);
renderer.setPixelRatio(window.devicePixelRatio);

// Camera setup
const aspectRatio = canvas.clientWidth / canvas.clientHeight;
const camera = new THREE.PerspectiveCamera(75, aspectRatio, 0.1, 1000);
camera.position.set(0.75, 1.5, 1.5); // Adjust camera position for better view
camera.lookAt(0, -0.75, 0);

// Light source for better visibility
const light = new THREE.PointLight(0xffffff, 1, 100);
light.position.set(5, 5, 5);
scene.add(light);

// Create a parent group for the shelf and rings
const shelfGroup = new THREE.Group();

// Extrusion settings
const extrudeSettings = {
    steps: 1,
    depth: 0.3, // Extrusion depth along Y-axis
    bevelEnabled: false,
};

// Square part
const squareShape = new THREE.Shape();
squareShape.moveTo(-1, -0.5);
squareShape.lineTo(1, -0.5);
squareShape.lineTo(1, 0.5);
squareShape.lineTo(-1, 0.5);
squareShape.lineTo(-1, -0.5);

// Left semicircle
const leftArcShape = new THREE.Shape();
leftArcShape.absarc(-1, 0, 0.5, Math.PI / 2, (3 * Math.PI) / 2, false);

// Right semicircle
const rightArcShape = new THREE.Shape();
rightArcShape.absarc(1, 0, 0.5, (3 * Math.PI) / 2, Math.PI / 2, false);

// Combine shapes into one geometry using ExtrudeGeometry
const squareGeometry = new THREE.ExtrudeGeometry(squareShape, extrudeSettings);
const leftArcGeometry = new THREE.ExtrudeGeometry(leftArcShape, extrudeSettings);
const rightArcGeometry = new THREE.ExtrudeGeometry(rightArcShape, extrudeSettings);

// Combine materials and meshes
const material = new THREE.MeshStandardMaterial({ color: 0x90ee90 });
const squareMesh = new THREE.Mesh(squareGeometry, material);
const leftArcMesh = new THREE.Mesh(leftArcGeometry, material);
const rightArcMesh = new THREE.Mesh(rightArcGeometry, material);

// Adjust positions
leftArcMesh.position.set(0, 0, 0);
rightArcMesh.position.set(0, 0, 0);

// Initial translations and rotations
squareMesh.rotation.x += Math.PI/2;
leftArcMesh.rotation.x += Math.PI/2;
rightArcMesh.rotation.x += Math.PI/2;

squareMesh.position.y = 0.15;
leftArcMesh.position.y = 0.15;
rightArcMesh.position.y = 0.15;

// Add the shelf components to the group
shelfGroup.add(squareMesh);
shelfGroup.add(leftArcMesh);
shelfGroup.add(rightArcMesh);

// Add LED rings (2D annuli)
const ringMaterial = new THREE.MeshStandardMaterial({ color: 0xffffff, side: THREE.DoubleSide });
const ringGeometry = new THREE.RingGeometry(0.42, 0.47, 32);

// Create and position rings
const rings = []; // Store references to rings for animation
for (let i = 0; i < 3; i++) {
    const ring = new THREE.Mesh(ringGeometry, ringMaterial);
    ring.position.set(-1 + i, 0.16, 0); // Stack rings slightly above each other
    ring.rotation.x = Math.PI / 2; // Align rings horizontally
    shelfGroup.add(ring); // Add each ring to the group
    rings.push(ring); // Store each ring in the array
}

// Add the group to the scene
scene.add(shelfGroup);


// Add a grid for reference
const gridHelper = new THREE.GridHelper(4, 10, 0x888888, 0x444444);
scene.add(gridHelper);

// Animation
function animate() {
    requestAnimationFrame(animate);

    // Rotate the shelf for visual effect
    shelfGroup.rotation.y += 0.005;

    renderer.render(scene, camera);
}
animate();

// Resize canvas dynamically on window resize
window.addEventListener('resize', () => {
    const width = canvas.clientWidth;
    const height = canvas.clientHeight;

    renderer.setSize(width, height);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
});
