from setuptools import setup, find_packages

setup(
    name="Path-Planning-for-Intelligent-Mobile-Robots",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        line.strip() for line in open("requirements.txt").readlines() if line.strip()
    ],
    extras_require={
        "dev": ["pytest", "black", "flake8", "mypy"],  # Development dependencies
        "ml": [
            "torch",
            "torchvision",
            "scikit-learn",
            "numpy",
            "pandas",
        ],  # Machine Learning dependencies
        "viz": ["opencv-python", "seaborn", "plotly"],  # Visualization dependencies
    },
    author="Dada Nanjesha Gouda Shanbog",
    description="A comprehensive Path Planning library implementing A*, D*, RRT*, Theta* and Hybrid DQN-A*.",
    url="https://github.com/DadaNanjesha/Path-Planning-for-Intelligent-Mobile-Robots",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.10",
)
