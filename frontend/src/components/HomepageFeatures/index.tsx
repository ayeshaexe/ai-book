import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: '🧠 From Physics to Intelligence',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Build robots from the ground up. Learn how humanoid robots perceive the world, simulate physics, and evolve from simple motion to intelligent behavior using ROS 2, Gazebo, and Isaac Sim.
      </>
    ),
  },
  {
    title: '🤖 Simulation-First Learning',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Train before touching hardware. Master digital twins, sensor simulation, and photorealistic environments to test perception, navigation, and manipulation safely and efficiently.
      </>
    ),
  },
  {
    title: '🗣️ Language Meets Action',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        When AI thinks and robots act. Explore Vision-Language-Action systems where voice commands, LLM planning, and robotic execution come together in a full humanoid capstone project.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
