#include "raylib.h"
#include "raymath.h"
#include "rk4.h"

// Sounds created in ChipTone by SFBGames embedded as code:
#include "lose.h"
#include "paddle.h"
#include "score.h"
#include "wall.h"
#include "win.h"

typedef struct Brick {
	Rectangle rect;
	Color color;
    bool isAlive;
    // Row and column could be inferred from position, but IMO better to be explicit here rather than rounding down floats.
    int rowIndex;
    int columnIndex;
} Brick;

// Falling debris and block.
typedef struct Particle
{
    Rectangle rect;
    Vector2 velocity;
    float angleDegrees;
    float angularVelocity; // degrees per second
    Color color;
} Particle;

typedef enum
{
    GAMESTATE_INITIAL,
    GAMESTATE_PLAYING,
    GAMESTATE_LOSS,
    GAMESTATE_WIN,
} GameState;

#define MAX_PARTICLE_COUNT 200
Particle particles[MAX_PARTICLE_COUNT]; // todo: this should probably be on the heap
int activeParticleCount;

Camera2D camera = { 0 };
Rk4State2 cameraShake;
GameState gameState = GAMESTATE_INITIAL;

Rectangle paddleRect;
int paddleClamped; // Whether paddle was at the left min / right max for purposes of screen shake.

Vector2 ballCenter;
const float ballRadius = 1.5f;
const float defaultBallSpeed = 40.0f;
float ballSpeed;
Vector2 ballDirection;
int consecutiveHitCount;
int totalHitCount;
bool hasReachedCeiling;

#define HORIZONTAL_BRICK_COUNT 14
#define VERTICAL_BRICK_COUNT 8
#define TOTAL_BRICK_COUNT HORIZONTAL_BRICK_COUNT * VERTICAL_BRICK_COUNT
Brick bricks[TOTAL_BRICK_COUNT];
int aliveBrickCount;

Sound paddleSound; // Played when ball hits paddle.
Sound scoreSound; // Played when ball hits brick.
Sound wallSound; // Played when ball hits wall.
Sound loseSound; // Played when player loses.
Sound winSound; // Played when player wins.

// todo: we should replace with something that has uniform distribution
float GetRandomFloatValue(float min, float max)
{
    return (float)GetRandomValue((int)min, (int)max);
}

// Z value of 3D cross product.
float Vector2CrossProduct(Vector2 v1, Vector2 v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

// Find the hit (if any) between a ray and circle.
bool RayCircleIntersection(Vector2 rayOrigin, Vector2 rayDirection, Vector2 circleCenter, float circleRadius, float *hitDistAlongRay)
{
    Vector2 circleRelativeToRay = Vector2Subtract(circleCenter, rayOrigin);
	float circleToRayDistSqr = Vector2LengthSqr(circleRelativeToRay);
	float circleRadiusSqr = circleRadius * circleRadius;
    if (circleToRayDistSqr < circleRadiusSqr)
    {
        // Ray origin is overlapping circle.
        *hitDistAlongRay = 0.0f;
        return true;
    }

    float circleCenterDistAlongRay = Vector2DotProduct(rayDirection, circleRelativeToRay);
    if (circleCenterDistAlongRay < 0.0f)
    {
        // Circle is behind ray.
        return false;
    }

    float distFromCircleToNearestPointOnRaySqr = circleToRayDistSqr - circleCenterDistAlongRay * circleCenterDistAlongRay;
    // We now have a right triangle where the hypotenuse is the distance between the ray origin and circle center, one of the sides is
    // the distance along the ray to the projected center, and the other side is the distance between the center and the ray. If the length
    // of this final side is less than the radius of the circle then the ray is pointing inside the circle.
    if (distFromCircleToNearestPointOnRaySqr <= circleRadiusSqr)
    {
        // We now have another right angle triangle. The hypotenuse is the circle radius, the known side length is the distance between the
        // circle center and the nearest point on the ray, so we can solve the distance between the nearest point on the ray and the hit.
        float distWithinCircle = sqrtf(circleRadiusSqr - distFromCircleToNearestPointOnRaySqr);
        *hitDistAlongRay = circleCenterDistAlongRay - distWithinCircle;
        return true;
    }
    else
    {
        // No intersection.
        return false;
    }
}

// Find the hit (if any) between a ray and infinite plane.
bool RayPlaneIntersection(Vector2 rayOrigin, Vector2 rayDirection, Vector2 planeOrigin, Vector2 planeNormal, float *hitDistAlongRay)
{
    float rayPlaneCosine = Vector2DotProduct(rayDirection, planeNormal);
    if (rayPlaneCosine >= 0.0f)
    {
        // Ray is either parallel to plane, facing away from plane, or passing through plane from the other side.
        return false;
    }

	// sohcahtoa -> cosine = adjacent/hypotenuse, so hypotenuse = adjacent/cosine
	// We have a right angle triangle where the adjacent is the line directly from the ray origin to the plane, and the angle is between the
	// plane normal and the ray direction, so we can solve for the hypotenuse - the distance from the ray origin to the plane.
    Vector2 planeRelativeToRay = Vector2Subtract(planeOrigin, rayOrigin);
    float rayDistAlongPlaneNormal = Vector2DotProduct(planeRelativeToRay, planeNormal);
    if (rayDistAlongPlaneNormal >= 0.0f)
    {
        // Ray is either touching or behind plane.
        return false;
    }
    
    // Both values are negative so we get a positive result.
    *hitDistAlongRay = rayDistAlongPlaneNormal / rayPlaneCosine;
    return true;
}

// Find the hit (if any) between a circle and infinite plane.
bool CirclePlaneIntersection(Vector2 circleOrigin, Vector2 circleDirection, float circleRadius, Vector2 planeOrigin, Vector2 planeNormal, float *hitDistAlongRay)
{
    planeOrigin.x += planeNormal.x * circleRadius;
    planeOrigin.y += planeNormal.y * circleRadius;
    return RayPlaneIntersection(circleOrigin, circleDirection, planeOrigin, planeNormal, hitDistAlongRay);
}

// Find the hit (if any) between a ray and line.
bool RayLineSegmentIntersection(Vector2 rayOrigin, Vector2 rayDirection, Vector2 lineStart, Vector2 lineEnd, float *hitDistAlongRay)
{
    Vector2 lineDelta = Vector2Subtract(lineEnd, lineStart);
    float lineLength = Vector2Length(lineDelta);
    if (lineLength <= 0.0f)
    {
        return false;
    }

    float inverseLineLength = 1.0f / lineLength;
    Vector2 lineDirection;
    lineDirection.x = lineDelta.x * inverseLineLength;
    lineDirection.y = lineDelta.y * inverseLineLength;

    float rayLineSine = Vector2CrossProduct(rayDirection, lineDirection);
    if (fabs(rayLineSine) < 0.000001f) // epsilon
    {
        // Ray and line are collinear or parallel.
        return false;
    }

	Vector2 lineRelativeToRay = Vector2Subtract(lineStart, rayOrigin);

    // sohcahtoa -> sine = opposite / hypotenuse, opposite = sine * hypotenuse, hypotenuse = opposite / sine
    float distFromRayOriginToLine = Vector2CrossProduct(lineRelativeToRay, lineDirection);
    float distAlongRayToLine = distFromRayOriginToLine / rayLineSine;
    if (distAlongRayToLine <= 0.0f)
    {
        // Ray is pointed away from line.
        return false;
    }

    // Project ray onto line.
    float rayDistAlongLine = Vector2CrossProduct(lineRelativeToRay, rayDirection) / rayLineSine;
    if (rayDistAlongLine < 0.0f || rayDistAlongLine > lineLength)
    {
        // Ray missed the line segment.
        return false;
    }

    *hitDistAlongRay = distAlongRayToLine;
    return true;
}

// Find the hit (if any) between a circle and a rectangle.
// Calculated as the intersection of a ray and an inflated rectangle with rounded corners. Maybe there is a better way to do this?
bool CircleRectIntersection(Vector2 circleOrigin, Vector2 circleDirection, float circleRadius, Rectangle rect, float *hitDistAlongRay, Vector2 *hitNormal)
{
	Vector2 nearestCornerCenter;
    Vector2 horizontalCornerCenter;
    Vector2 verticalCornerCenter;
    float verticalLineNormal;
    float horizontalLineNormal;

    if (circleDirection.x > 0.0f)
    {
        if (circleDirection.y > 0.0f)
        {
            // upper-left
            nearestCornerCenter.x = rect.x;
            nearestCornerCenter.y = rect.y;
            horizontalCornerCenter.x = rect.x + rect.width;
			horizontalCornerCenter.y = rect.y;
            verticalCornerCenter.x = rect.x;
            verticalCornerCenter.y = rect.y + rect.height;

            horizontalLineNormal = -1.0f;
        }
        else
        {
			// lower-left
			nearestCornerCenter.x = rect.x;
			nearestCornerCenter.y = rect.y + rect.height;
			horizontalCornerCenter.x = rect.x + rect.width;
			horizontalCornerCenter.y = rect.y + rect.height;
			verticalCornerCenter.x = rect.x;
			verticalCornerCenter.y = rect.y;

			horizontalLineNormal = 1.0f;
        }

        verticalLineNormal = -1.0f;
    }
    else
    {
        if (circleDirection.y > 0.0f)
        {
            // upper-right
			nearestCornerCenter.x = rect.x + rect.width;
			nearestCornerCenter.y = rect.y;
			horizontalCornerCenter.x = rect.x;
			horizontalCornerCenter.y = rect.y;
			verticalCornerCenter.x = rect.x + rect.width;
			verticalCornerCenter.y = rect.y + rect.height;

			horizontalLineNormal = -1.0f;
        }
        else
        {
            // lower-right
			nearestCornerCenter.x = rect.x + rect.width;
			nearestCornerCenter.y = rect.y + rect.height;
			horizontalCornerCenter.x = rect.x;
			horizontalCornerCenter.y = rect.y + rect.height;
			verticalCornerCenter.x = rect.x + rect.width;
			verticalCornerCenter.y = rect.y;

			horizontalLineNormal = 1.0f;
        }

        verticalLineNormal = 1.0f;
    }

    bool hitAnything = false;
    float nearestDist = 1000.0f;

    if (RayCircleIntersection(circleOrigin, circleDirection, nearestCornerCenter, circleRadius, hitDistAlongRay) && *hitDistAlongRay < nearestDist)
    {
        nearestDist = *hitDistAlongRay;

        if (fabs(circleOrigin.x - nearestCornerCenter.x) < fabs(circleOrigin.y - nearestCornerCenter.y))
        {
            // Hit was nearest the top or bottom of the circle.
			hitNormal->x = 0.0f;
            hitNormal->y = horizontalLineNormal;
        }
        else
		{
			// Hit was nearest the left or right of the circle.
			hitNormal->x = verticalLineNormal;
            hitNormal->y = 0.0f;
        }

        hitAnything = true;
    }

	if (RayCircleIntersection(circleOrigin, circleDirection, verticalCornerCenter, circleRadius, hitDistAlongRay) && *hitDistAlongRay < nearestDist)
	{
		nearestDist = *hitDistAlongRay;
		hitNormal->x = verticalLineNormal;
        hitNormal->y = 0.0f;
		hitAnything = true;
	}

	if (RayCircleIntersection(circleOrigin, circleDirection, horizontalCornerCenter, circleRadius, hitDistAlongRay) && *hitDistAlongRay < nearestDist)
	{
		nearestDist = *hitDistAlongRay;
		hitNormal->x = 0.0f;
        hitNormal->y = horizontalLineNormal;
		hitAnything = true;
	}

	// Left or right side of the rect
    Vector2 verticalLineStart = nearestCornerCenter;
    verticalLineStart.x += verticalLineNormal * circleRadius;
	Vector2 verticalLineEnd = verticalCornerCenter;
    verticalLineEnd.x += verticalLineNormal * circleRadius;
    if (RayLineSegmentIntersection(circleOrigin, circleDirection, verticalLineStart, verticalLineEnd, hitDistAlongRay) && *hitDistAlongRay < nearestDist)
    {
        nearestDist = *hitDistAlongRay;
		hitNormal->x = verticalLineNormal;
		hitNormal->y = 0.0f;
        hitAnything = true;
    }

	// Top or bottom side of the rect
	Vector2 horizontalLineStart = nearestCornerCenter;
    horizontalLineStart.y += horizontalLineNormal * circleRadius;
	Vector2 horizontalLineEnd = horizontalCornerCenter;
    horizontalLineEnd.y += horizontalLineNormal * circleRadius;
	if (RayLineSegmentIntersection(circleOrigin, circleDirection, horizontalLineStart, horizontalLineEnd, hitDistAlongRay) && *hitDistAlongRay < nearestDist)
	{
        nearestDist = *hitDistAlongRay;
        hitNormal->x = 0.0f;
        hitNormal->y = horizontalLineNormal;
        hitAnything = true;
    }

    *hitDistAlongRay = nearestDist;
    return hitAnything;
}

// Get a particle from the pool, or null if pool is full.
Particle *SpawnParticle()
{
    if (activeParticleCount < MAX_PARTICLE_COUNT)
    {
        Particle *next_particle = &particles[activeParticleCount];
		next_particle->velocity.x = 0.0f;
		next_particle->velocity.y = 0.0f;
        next_particle->angleDegrees = 0.0f;
        next_particle->angularVelocity = 0.0f;
        ++activeParticleCount;
        return next_particle;
    }
    else
    {
        return NULL;
    }
}

void ShakeCamera(float scale)
{
    cameraShake.position.x += ballDirection.x * ballSpeed * -0.01f * scale;
    cameraShake.position.y += ballDirection.y * ballSpeed * -0.01f * scale;
}

// Called when the game starts up and when resetting after a win/loss.
void InitializeGameState()
{
	cameraShake = CLITERAL(Rk4State2) { 0 };

	paddleRect = CLITERAL(Rectangle)
	{
		.x = 34.0f, // Intentionally off-center forcing player to choose an angle (rather than straight up), or default to up-right.
		.y = 96.0f,
		.width = 16.0f,
		.height = 1.0f,
	};
	paddleClamped = 0;

	ballCenter = CLITERAL(Vector2)
	{
		.x = 50.0f,
		.y = 50.0f,
	};

	ballDirection = CLITERAL(Vector2) { 0.0f, 1.0f };
	ballSpeed = defaultBallSpeed;

	consecutiveHitCount = 0;
	totalHitCount = 0;
	hasReachedCeiling = false;

	activeParticleCount = 0;

	const Color rowColors[8] =
	{
		CLITERAL(Color) { 228, 69, 47, 255 }, // Red
		CLITERAL(Color) { 218, 62, 50, 255 }, // Darker red
		CLITERAL(Color) { 239, 131, 26, 255 }, // Orange
		CLITERAL(Color) { 231, 117, 25, 255 }, // Darker orange
		CLITERAL(Color) { 82, 167, 63, 255 }, // Green
		CLITERAL(Color) { 73, 147, 67, 255 }, // Darker green
		CLITERAL(Color) { 239, 219, 53, 255 }, // Yellow,
		CLITERAL(Color) { 226, 190, 48, 255 }, // Darker yellow,
	};

	int createBrickIndex = 0;
	float brickWidth = 100.0f / HORIZONTAL_BRICK_COUNT;
	float brickHeight = 40.0f / VERTICAL_BRICK_COUNT;
	for (int columnIndex = 0; columnIndex < HORIZONTAL_BRICK_COUNT; ++columnIndex)
	{
		for (int rowIndex = 0; rowIndex < VERTICAL_BRICK_COUNT; ++rowIndex)
		{
			bricks[createBrickIndex].rect.x = columnIndex * brickWidth + 0.5f;
			bricks[createBrickIndex].rect.y = 7.0f + rowIndex * brickHeight;
			bricks[createBrickIndex].rect.width = brickWidth - 1.0f;
			bricks[createBrickIndex].rect.height = brickHeight - 1.0f;
			bricks[createBrickIndex].color = rowColors[rowIndex / 2 * 2 + (rowIndex + columnIndex) % 2];
			bricks[createBrickIndex].rowIndex = rowIndex;
			bricks[createBrickIndex].columnIndex = columnIndex;
			bricks[createBrickIndex].isAlive = true;
			++createBrickIndex;
		}
	}
	aliveBrickCount = createBrickIndex;

	gameState = GAMESTATE_PLAYING;
}

// Called during startup, win, and lose screens.
void RunLobbyFrame(const char *headerText, const char* bodyText)
{
	if (IsKeyReleased(KEY_SPACE))
	{
		InitializeGameState();
	}

	int width = GetScreenWidth();
	int height = GetScreenHeight();
	const int headerFontSize = 40;
	const int bodyFontSize = 20;
	DrawText(headerText, width / 2 - MeasureText(headerText, headerFontSize) / 2, height / 2 - 30, headerFontSize, RAYWHITE);
	DrawText(bodyText, width / 2 - MeasureText(bodyText, bodyFontSize) / 2, height / 2 + 30, bodyFontSize, RAYWHITE);

	DrawText("Credits: Nelson Sexton, raylib by @raysan5, ChipTone by SFB Games, and special thanks to Glenn Fiedler", 5, height - 15, 10, GRAY);
}

// Called each frame during gameplay.
void RunGameFrame()
{
	float deltaSeconds = GetFrameTime();

	int windowWidth = GetScreenWidth();
	int windowHeight = GetScreenHeight();
	// We use camera to get a full resolution square world regardless of aspect ratio.
	int minWindowSize = windowWidth < windowHeight ? windowWidth : windowHeight;
	camera.zoom = (float) minWindowSize / 100.0f;
	int scissorOffsetX = (windowWidth - minWindowSize) / 2;
	int scissorOffsetY = (windowHeight - minWindowSize) / 2;

	BeginScissorMode(scissorOffsetX, scissorOffsetY, minWindowSize, minWindowSize); // Clip anything outside world square.
	ClearBackground(DARKGRAY);

	// Update camera *before* spring to make sure full shake from previous frame is applied.
	camera.target.x = cameraShake.position.x;
	camera.target.y = cameraShake.position.y;
	Rk4Integrate(&cameraShake, deltaSeconds);
	camera.offset.x = (float)scissorOffsetX;
	camera.offset.y = (float)scissorOffsetY;

	BeginMode2D(camera);

	if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))
	{
		paddleRect.x -= deltaSeconds * 50.0f;
	}
	else if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT))
	{
		paddleRect.x += deltaSeconds * 50.0f;
	}
	if (paddleRect.x < 0.0f)
	{
		paddleRect.x = 0.0f;
		if (paddleClamped > -1)
		{
			paddleClamped = -1;
			cameraShake.position.x -= 0.5f;
		}
	}
	else if (paddleRect.x > 100.0f - paddleRect.width)
	{
		paddleRect.x = 100.0f - paddleRect.width;
		if (paddleClamped < 1)
		{
			paddleClamped = 1;
			cameraShake.position.x += 0.5f;
		}
	}
	else
	{
		paddleClamped = 0;
	}

	DrawRectangleRec(paddleRect, RAYWHITE);

	Vector2 worldUpperLeft = { 0 };
	Vector2 worldBottomRight =
	{
		.x = 100.0f,
		.y = 100.0f,
	};
	Vector2 topPlaneNormal =
	{
		.x = 0.0f,
		.y = 1.0f,
	};
	Vector2 leftPlaneNormal =
	{
		.x = 1.0f,
		.y = 0.0f,
	};
	Vector2 rightPlaneNormal =
	{
		.x = -1.0f,
		.y = 0.0f,
	};

	// Continuous collision detection for ball. Prevents traveling through blocks at low frame rate.
	float simulationTime = deltaSeconds;
	while (simulationTime > 0.0f)
	{
		float moveDistance = ballSpeed * simulationTime;

		// todo: separate collision sweep and result (e.g. hit type enum) from handling of result 
		bool hitAnything = false;
		float nearestDistance = moveDistance;
		Vector2 hitNormal = { 0 };
		Brick* hitBrick = NULL;
		bool hitPaddle = false;
		bool hitTopPlane = false;
		float tempHitDistance;
		Vector2 tempHitNormal;
		if (CirclePlaneIntersection(ballCenter, ballDirection, ballRadius, worldUpperLeft, topPlaneNormal, &tempHitDistance) && tempHitDistance < nearestDistance)
		{
			hitAnything = true;
			nearestDistance = tempHitDistance;
			hitNormal = topPlaneNormal;
			hitTopPlane = true;
		}
		if (CirclePlaneIntersection(ballCenter, ballDirection, ballRadius, worldUpperLeft, leftPlaneNormal, &tempHitDistance) && tempHitDistance < nearestDistance)
		{
			hitAnything = true;
			nearestDistance = tempHitDistance;
			hitNormal = leftPlaneNormal;
			hitTopPlane = false;
		}
		if (CirclePlaneIntersection(ballCenter, ballDirection, ballRadius, worldBottomRight, rightPlaneNormal, &tempHitDistance) && tempHitDistance < nearestDistance)
		{
			hitAnything = true;
			nearestDistance = tempHitDistance;
			hitNormal = rightPlaneNormal;
			hitTopPlane = false;
		}

		for (int brickIndex = 0; brickIndex < TOTAL_BRICK_COUNT; ++brickIndex)
		{
			Brick* brick = &bricks[brickIndex];
			if (!brick->isAlive)
				continue;

			if (CircleRectIntersection(ballCenter, ballDirection, ballRadius, brick->rect, &tempHitDistance, &tempHitNormal) && tempHitDistance < nearestDistance)
			{
				hitAnything = true;
				nearestDistance = tempHitDistance;
				hitBrick = brick;
				hitNormal = tempHitNormal;
				hitTopPlane = false;
			}
		}

		// Only consider downward velocity to prevent ball getting stuck on paddle after bounce.
		if (ballDirection.y > 0.0f && CircleRectIntersection(ballCenter, ballDirection, ballRadius, paddleRect, &tempHitDistance, &tempHitNormal) && tempHitDistance < nearestDistance)
		{
			hitAnything = true;
			nearestDistance = tempHitDistance;
			hitNormal = tempHitNormal;
			hitBrick = NULL;
			hitPaddle = true;
			hitTopPlane = false;
		}

		if (!hitAnything)
		{
			ballCenter = Vector2Add(ballCenter, Vector2Scale(ballDirection, moveDistance));
			break;
		}

		if (hitBrick)
		{
			hitBrick->isAlive = false;
			--aliveBrickCount;
			ShakeCamera(2.0f);

			Particle* particle = SpawnParticle();
			if (particle)
			{
				particle->rect = hitBrick->rect;
				particle->rect.x += particle->rect.width * 0.5f;
				particle->rect.y += particle->rect.height * 0.5f;
				particle->velocity.x = ballDirection.x * ballSpeed;
				particle->velocity.y = ballDirection.y * ballSpeed;

				float angularDir;
				if (ballCenter.x < hitBrick->rect.x + hitBrick->rect.width * 0.5f)
				{
					// Hit left side
					if (ballCenter.y < hitBrick->rect.y + hitBrick->rect.height * 0.5f)
					{
						// Hit top
						angularDir = -1.0f; // clockwise
					}
					else
					{
						// Hit bottom
						angularDir = 1.0f; // counter-clockwise
					}
				}
				else
				{
					// Hit right side
					if (ballCenter.y < hitBrick->rect.y + hitBrick->rect.height * 0.5f)
					{
						// Hit top
						angularDir = 1.0f; // counter-clockwise
					}
					else
					{
						// Hit bottom
						angularDir = -1.0f; // clockwise
					}
				}
				particle->angularVelocity = angularDir * GetRandomFloatValue(60.0f, 120.0f);

				particle->color = hitBrick->color;

				for (int dustParticleIndex = GetRandomValue(5, 10); dustParticleIndex >= 0; --dustParticleIndex)
				{
					Particle* dustParticle = SpawnParticle();
					if (!dustParticle)
					{
						// Particle pool is full.
						break;
					}

					dustParticle->rect.x = GetRandomFloatValue(hitBrick->rect.x, hitBrick->rect.x + hitBrick->rect.width);
					dustParticle->rect.y = GetRandomFloatValue(hitBrick->rect.y, hitBrick->rect.y + hitBrick->rect.height);
					dustParticle->rect.width = 1.0f;
					dustParticle->rect.height = 1.0f;
					dustParticle->velocity.x = ballDirection.x * ballSpeed * -0.25f + GetRandomFloatValue(-5.0f, 5.0f);
					dustParticle->velocity.y = ballDirection.y * ballSpeed * -0.25f + GetRandomFloatValue(-5.0f, 5.0f);
					dustParticle->angleDegrees = 0.0f;
					dustParticle->angularVelocity = 0.0f;
					dustParticle->color = LIGHTGRAY;
					dustParticle->color.a = 63;
				}
			}

			SetSoundPitch(scoreSound, 1.0f + consecutiveHitCount * 0.1f);
			++consecutiveHitCount;
			++totalHitCount;
			PlaySound(scoreSound);

			float newBallSpeed = ballSpeed;
			if (hitBrick->rowIndex < 2)
			{
				newBallSpeed = defaultBallSpeed + 20.0f;
			}
			else if (hitBrick->rowIndex < 4)
			{
				newBallSpeed = defaultBallSpeed + 15.0f;
			}
			else if (totalHitCount == 12)
			{
				newBallSpeed = defaultBallSpeed + 10.0f;
			}
			else if (totalHitCount == 4)
			{
				newBallSpeed = defaultBallSpeed + 5.0f;
			}
			ballSpeed = fmaxf(ballSpeed, newBallSpeed);
		}
		else
		{
			ShakeCamera(1.0f);
		}

		if (hitTopPlane && !hasReachedCeiling)
		{
			// Shrink paddle in half horizontally.
			hasReachedCeiling = true;
			paddleRect.width *= 0.5f;
			paddleRect.x += paddleRect.width * 0.5f;
		}

		ballCenter = Vector2Add(ballCenter, Vector2Scale(ballDirection, nearestDistance));
		simulationTime *= nearestDistance / moveDistance;

		if (hitPaddle)
		{
			float t = Clamp((ballCenter.x - paddleRect.x) / paddleRect.width, 0.0f, 1.0f);
			float angle = Lerp(PI * 0.75f, PI * 0.25f, t);
			ballDirection.x = cosf(angle);
			ballDirection.y = -sinf(angle);

			consecutiveHitCount = 0;
			SetSoundPitch(paddleSound, GetRandomFloatValue(0.9f, 1.1f));
			PlaySound(paddleSound);
		}
		else
		{
			ballDirection = Vector2Reflect(ballDirection, hitNormal);
		}

		if (!hitPaddle && !hitBrick)
		{
			SetSoundPitch(wallSound, GetRandomFloatValue(0.9f, 1.1f));
			PlaySound(wallSound);
		}
	}

	if (aliveBrickCount < 1)
	{
		gameState = GAMESTATE_WIN;
		PlaySound(winSound);
	}
	else if (ballCenter.y > 100.0f)
	{
		gameState = GAMESTATE_LOSS;
		PlaySound(loseSound);
	}

	for (int brickIndex = 0; brickIndex < TOTAL_BRICK_COUNT; ++brickIndex)
	{
		Brick* brick = &bricks[brickIndex];
		if (!brick->isAlive)
			continue;

		DrawRectangleRec(brick->rect, brick->color);
	}

	DrawCircleV(ballCenter, ballRadius, RAYWHITE);

	for (int particleIndex = activeParticleCount - 1; particleIndex >= 0; --particleIndex)
	{
		Particle* particle = &particles[particleIndex];

		if (particle->rect.x < -10.0f || particle->rect.x > 110.0f || particle->rect.y > 110.0f)
		{
			// Remove particles outside the world from the pool, allowing them to be recycled.
			--activeParticleCount;
			particles[particleIndex] = particles[activeParticleCount];
			continue;
		}

		Vector2 origin =
		{
			particle->rect.width * 0.5f,
			particle->rect.height * 0.5f
		};
		DrawRectanglePro(particle->rect, origin, particle->angleDegrees, particle->color);
		particle->rect.x += particle->velocity.x * deltaSeconds;
		particle->rect.y += particle->velocity.y * deltaSeconds;
		particle->angleDegrees += particle->angularVelocity * deltaSeconds;
		particle->velocity.y += 200.0f * deltaSeconds;
	}

	EndMode2D();
	EndScissorMode();
}

// Load sounds embedded as code. Created in ChipTone by SFBGames.
void LoadSounds() 
{
	Wave loseWave =
	{
		.frameCount = LOSE_FRAME_COUNT,
		.sampleRate = LOSE_SAMPLE_RATE,
		.sampleSize = LOSE_SAMPLE_SIZE,
		.channels = LOSE_CHANNELS,
		.data = loseData
	};
	loseSound = LoadSoundFromWave(loseWave);

	Wave paddleWave =
	{
		.frameCount = PADDLE_FRAME_COUNT,
		.sampleRate = PADDLE_SAMPLE_RATE,
		.sampleSize = PADDLE_SAMPLE_SIZE,
		.channels = PADDLE_CHANNELS,
		.data = paddleData
	};
	paddleSound = LoadSoundFromWave(paddleWave);

	Wave scoreWave =
	{
		.frameCount = SCORE_FRAME_COUNT,
		.sampleRate = SCORE_SAMPLE_RATE,
		.sampleSize = SCORE_SAMPLE_SIZE,
		.channels = SCORE_CHANNELS,
		.data = scoreData
	};
	scoreSound = LoadSoundFromWave(scoreWave);

	Wave wallWave =
	{
		.frameCount = WALL_FRAME_COUNT,
		.sampleRate = WALL_SAMPLE_RATE,
		.sampleSize = WALL_SAMPLE_SIZE,
		.channels = WALL_CHANNELS,
		.data = wallData
	};
	wallSound = LoadSoundFromWave(wallWave);

	Wave winWave =
	{
		.frameCount = WIN_FRAME_COUNT,
		.sampleRate = WIN_SAMPLE_RATE,
		.sampleSize = WIN_SAMPLE_SIZE,
		.channels = WIN_CHANNELS,
		.data = winData
	};
	winSound = LoadSoundFromWave(winWave);
}

// Entry point.
int main()
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    SetTargetFPS(500);
    InitWindow(600, 600, "Brick Breakers");

    InitAudioDevice();
	LoadSounds();

    while (!WindowShouldClose())
    {
		BeginDrawing();
		ClearBackground(BLACK);

		switch (gameState)
		{
			case GAMESTATE_INITIAL:
				RunLobbyFrame("Brick Breakers", "Press spacebar...");
				break;

			case GAMESTATE_PLAYING:
				RunGameFrame();
				break;

			case GAMESTATE_LOSS:
				RunLobbyFrame("Loss :(", "Press spacebar to try again...");
				break;

			case GAMESTATE_WIN:
				RunLobbyFrame("Win :)", "Press spacebar to play again...");
				break;
		}

        EndDrawing();
    }

    CloseAudioDevice();
    CloseWindow();

    return 0;
}
