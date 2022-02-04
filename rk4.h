#pragma once

// Runge Kutta order 4 integrator thanks to Glenn Fiedler
// https://gafferongames.com/post/integration_basics/

// 2D RK4 spring
typedef struct Rk4State2
{
	Vector2 position;
	Vector2 velocity;
} Rk4State2;

// 2D internal RK4 values
typedef struct Rk4Derivative2
{
	Vector2 velocity;
	Vector2 acceleration;
} Rk4Derivative2;

Vector2 Rk4CalculateAcceleration(Rk4State2 state)
{
	// todo: make the spring constants configurable
	const float k = 100.0f;
	const float b = 5.0f;
	Vector2 output;
	output.x = -k * state.position.x - b * state.velocity.x;
	output.y = -k * state.position.y - b * state.velocity.y;
	return output;
}

Rk4Derivative2 Rk4Evaluate(Rk4State2 initial, float deltaSeconds, Rk4Derivative2 delta)
{
	Rk4State2 state;
	state.position.x = initial.position.x + delta.velocity.x * deltaSeconds;
	state.position.y = initial.position.y + delta.velocity.y * deltaSeconds;
	state.velocity.x = initial.velocity.x + delta.acceleration.x * deltaSeconds;
	state.velocity.y = initial.velocity.y + delta.acceleration.y * deltaSeconds;

	Rk4Derivative2 output;
	output.velocity = state.velocity;
	output.acceleration = Rk4CalculateAcceleration(state);
	return output;
}

void Rk4Integrate(Rk4State2* state, float deltaSeconds)
{
	Rk4Derivative2 a, b, c, d;
	a = Rk4Evaluate(*state, deltaSeconds, CLITERAL(Rk4Derivative2) { 0 });
	b = Rk4Evaluate(*state, deltaSeconds * 0.5f, a);
	c = Rk4Evaluate(*state, deltaSeconds * 0.5f, b);
	d = Rk4Evaluate(*state, deltaSeconds, c);

	Vector2 velocity =
	{
		(a.velocity.x + 2.0f * (b.velocity.x + c.velocity.x) + d.velocity.x) / 6.0f,
		(a.velocity.y + 2.0f * (b.velocity.y + c.velocity.y) + d.velocity.y) / 6.0f
	};

	Vector2 acceleration =
	{
		(a.acceleration.x + 2.0f * (b.acceleration.x + c.acceleration.x) + d.acceleration.x) / 6.0f,
		(a.acceleration.y + 2.0f * (b.acceleration.y + c.acceleration.y) + d.acceleration.y) / 6.0f
	};

	state->position.x += velocity.x * deltaSeconds;
	state->position.y += velocity.y * deltaSeconds;
	state->velocity.x += acceleration.x * deltaSeconds;
	state->velocity.y += acceleration.y * deltaSeconds;
}
