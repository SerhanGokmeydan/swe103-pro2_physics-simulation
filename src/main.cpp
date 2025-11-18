#include "../include/physics/physics.hpp"
#include "../include/graphs/spring.hpp"
#include "../include/graphs/soft-body.hpp"
#include <optional>
#include <random>

std::vector<graphs::Ball> graphs::Balls;
std::vector<graphs::Spring> graphs::Springs;
std::vector<graphs::SoftBody> graphs::SoftBodys;

// creates random number
float getRandomNumber(float min, float max)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> distrib(min, max);

    return distrib(gen);
}

int main()
{
    int selectedBall = -1;
    int selectedBody = -1;
    int numberOfBalls = 5;
    int numberOfSoftBodys = 2;

    // set anti aliasing level
    sf::ContextSettings settings;
    settings.antiAliasingLevel = 16;

    // create the window
    sf::RenderWindow window(sf::VideoMode({1200, 900}), "My window", sf::Style::Close, sf::State::Windowed, settings);
    window.setVerticalSyncEnabled(true);

    // creates random balls
    for (int i = 0; i < numberOfBalls; i++)
    {
        float red = getRandomNumber(0.f, 255.f);   // red
        float green = getRandomNumber(0.f, 255.f); // green
        float blue = getRandomNumber(0.f, 255.f);  // blue
        float x = getRandomNumber(0.f, 1200.f);    // x position
        float y = getRandomNumber(0.f, 900.f);     // y position
        float r = getRandomNumber(30.f, 50.f);     // radius
        float m = getRandomNumber(10.f, 20.f);     // mass
        float e = getRandomNumber(0.1f, 0.6f);     // elasticity

        graphs::Balls.emplace_back(linalg::Vector(x, y), sf::Color(red, green, blue), r, m, e); // position, color, radius, mass, elasticity
    }

    // creates random bodys
    for (int i = 0; i < numberOfSoftBodys; i++)
    {
        float red = getRandomNumber(0.f, 255.f);   // red
        float green = getRandomNumber(0.f, 255.f); // green
        float blue = getRandomNumber(0.f, 255.f);  // blue
        float x = getRandomNumber(0.f, 1200.f);    // x position
        float y = getRandomNumber(0.f, 900.f);     // y position
        float r = getRandomNumber(50.f, 70.f);     // radius
        float m = getRandomNumber(10.f, 20.f);     // mass
        float e = getRandomNumber(0.1f, 0.5f);     // elasticity
        float s = getRandomNumber(0.1f, 0.8f);     // spring stiffness
        float p = getRandomNumber(0.f, 0.05f);      // pressure stiffness

        graphs::SoftBodys.emplace_back(linalg::Vector(x, y), sf::Color(red, green, blue), 25, r, m, e, s, p); // position, color, radius, mass, elasticity
    }

    graphs::Springs.emplace_back(graphs::Balls.at(0), graphs::Balls.at(1), 200.f, 0.5f);

    // run the program as long as the window is open
    while (window.isOpen())
    {
        // check all the window's events that were triggered since the last iteration of the loop
        while (const std::optional event = window.pollEvent())
        {
            // "close requested" event: we close the window
            if (event->is<sf::Event::Closed>())
            {
                window.close();
            }
        }

        bool mousePressed = sf::Mouse::isButtonPressed(sf::Mouse::Button::Left);
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        linalg::Vector mouseVector(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));

        // Sub-Steps
        for (int step = 0; step < physics::SUB_STEPS; step++)
        {
            // resets forces
            for (graphs::SoftBody &body : graphs::SoftBodys)
            {
                for (graphs::Ball &ball : body.cornerBalls)
                {
                    ball.springForce = linalg::Vector(0.f, 0.f);
                    ball.pressureForce = linalg::Vector(0.f, 0.f);
                }
            }
            for (graphs::Ball &ball : graphs::Balls)
            {
                ball.springForce = linalg::Vector(0.f, 0.f);
                ball.pressureForce = linalg::Vector(0.f, 0.f);
            }

            // computes forces
            for (graphs::SoftBody &body : graphs::SoftBodys)
            {
                for (graphs::Spring &spring : body.edgeSprings)
                {
                    spring.computeSpringForce();
                }
                body.computePressureForce();
            }
            for (graphs::Spring &spring : graphs::Springs)
            {
                spring.computeSpringForce();
            }

            // physics
            // update soft body
            for (graphs::SoftBody &body : graphs::SoftBodys)
            {
                for (graphs::Ball &ball : body.cornerBalls)
                {
                    ball.computeDragForce();
                    ball.computeFrictionForce();
                    ball.checkWallCollision();
                    ball.update(physics::SUB_DELTA_TIME);
                }

                body.update(); // update center of body
            }

            // update balls
            for (graphs::Ball &ball : graphs::Balls)
            {
                ball.computeDragForce();
                ball.computeFrictionForce();
                ball.checkWallCollision();
                ball.update(physics::SUB_DELTA_TIME);
            }

            // controls of collisions

            // ball vs ball
            for (int i = 0; i < graphs::Balls.size(); i++)
            {
                for (int j = i + 1; j < graphs::Balls.size(); j++)
                {
                    graphs::Balls.at(i).checkBallCollision(graphs::Balls.at(j));
                }
            }

            // ball vs body
            for (graphs::Ball &looseBall : graphs::Balls)
            {
                for (graphs::SoftBody &body : graphs::SoftBodys)
                {
                    // ball vs ball of softbody
                    for (graphs::Ball &cornerBall : body.cornerBalls)
                    {
                        looseBall.checkBallCollision(cornerBall);
                    }
                    // ball vs spring of softbody
                    for (graphs::Spring &spring : body.edgeSprings)
                    {
                        looseBall.checkSpringCollision(spring);
                    }
                }
            }

            // balls vs springs
            for (graphs::Ball &ball : graphs::Balls)
            {
                for (graphs::Spring &spring : graphs::Springs)
                {
                    ball.checkSpringCollision(spring);
                }
            }

            // balls of body vs springs
            for (graphs::SoftBody &body : graphs::SoftBodys)
            {
                for (graphs::Ball &cornerBall : body.cornerBalls)
                {
                    for (graphs::Spring &freeSpring : graphs::Springs)
                    {
                        cornerBall.checkSpringCollision(freeSpring);
                    }
                }
            }

            // body vs body
            for (int i = 0; i < graphs::SoftBodys.size(); i++)
            {
                graphs::SoftBody &bodyA = graphs::SoftBodys[i];

                for (int j = i + 1; j < graphs::SoftBodys.size(); j++)
                {
                    graphs::SoftBody &bodyB = graphs::SoftBodys[j];

                    // balls of body1 vs balls of body2
                    for (graphs::Ball &ballA : bodyA.cornerBalls)
                    {
                        for (graphs::Ball &ballB : bodyB.cornerBalls)
                        {
                            ballA.checkBallCollision(ballB);
                        }
                    }

                    // balls of body1 vs springs of body2
                    for (graphs::Ball &ballA : bodyA.cornerBalls)
                    {
                        for (graphs::Spring &springB : bodyB.edgeSprings)
                        {
                            ballA.checkSpringCollision(springB);
                        }
                    }

                    // balls of body2 vs springs of body1
                    for (graphs::Ball &ballB : bodyB.cornerBalls)
                    {
                        for (graphs::Spring &springA : bodyA.edgeSprings)
                        {
                            ballB.checkSpringCollision(springA);
                        }
                    }
                }
            }

            // SoftBody'nin Kendi İç Çarpışmaları (Top vs Top)
            for (graphs::SoftBody &body : graphs::SoftBodys)
            {
                for (int i = 0; i < body.cornerBalls.size(); i++)
                {
                    graphs::Ball &ball1 = body.cornerBalls[i];
                    for (int j = i + 1; j < body.cornerBalls.size(); j++)
                    {
                        graphs::Ball &ball2 = body.cornerBalls[j];
                        ball1.checkBallCollision(ball2);
                    }
                }
            }

        } // end of substeps

        // mouse control

        // for array of soft bodys
        for (int i = 0; i < graphs::SoftBodys.size(); i++)
        {
            if (mousePressed)
            {
                // Sadece başka bir top seçili değilse body'yi seç
                if (selectedBall == -1)
                {
                    float distance = (graphs::SoftBodys.at(i).center - mouseVector).magnitude();
                    if (distance < graphs::SoftBodys.at(i).radius)
                    {
                        selectedBody = i;
                    }
                }
            }
        }
        if (mousePressed && selectedBody != -1)
        {
            graphs::SoftBodys.at(selectedBody).projectileMotion(mouseVector, physics::FIXED_DELTA_TIME);
        }
        else if (selectedBody != -1 && graphs::SoftBodys.at(selectedBody).isBeingDragged)
        {
            graphs::SoftBodys.at(selectedBody).isBeingDragged = false;
            for (graphs::Ball &ball : graphs::SoftBodys.at(selectedBody).cornerBalls)
            {
                ball.isBeingDragged = false;
            }
            selectedBody = -1;
        }

        // for array of balls
        for (int i = 0; i < graphs::Balls.size(); i++)
        {
            if (mousePressed)
            {
                // Sadece bir body seçili değilse topu seç
                if (selectedBody == -1)
                {
                    float distance = (graphs::Balls.at(i).pos - mouseVector).magnitude();
                    if (distance < graphs::Balls.at(i).radius)
                    {
                        selectedBall = i;
                    }
                }
            }
        }
        if (mousePressed && selectedBall != -1)
        {
            graphs::Balls.at(selectedBall).projectileMotion(mouseVector, physics::FIXED_DELTA_TIME);
        }
        else if (selectedBall != -1 && graphs::Balls.at(selectedBall).isBeingDragged)
        {
            graphs::Balls.at(selectedBall).isBeingDragged = false;
            selectedBall = -1;
        }

        // drawings
        window.clear(sf::Color::Black);
        for (graphs::Ball &ball : graphs::Balls)
        {
            ball.draw(window);
        }
        for (graphs::Spring &spring : graphs::Springs)
        {
            spring.draw(window);
        }
        for (graphs::SoftBody &body : graphs::SoftBodys)
        {
            // body.draw(window);
            for (graphs::Ball &ball : body.cornerBalls)
            {
                ball.draw(window);
            }
            for (graphs::Spring &spring : body.edgeSprings)
            {
                spring.draw(window);
            }
        }
        window.display(); // end the current frame
    }
}