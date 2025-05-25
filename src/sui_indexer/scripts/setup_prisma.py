#!/usr/bin/env python3
import os
import subprocess
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class PrismaSetupNode(Node):
    def __init__(self):
        super().__init__('prisma_setup')
        
        try:
            # Get package paths
            self.pkg_share = get_package_share_directory('sui_indexer')
            self.prisma_path = os.path.join(self.pkg_share, 'prisma')
            
            # Ensure prisma directory exists
            if not os.path.exists(self.prisma_path):
                raise FileNotFoundError(f"Prisma schema directory not found at {self.prisma_path}")
            
            # Set up database path
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            data_dir = os.path.join(pkg_dir, 'data')
            os.makedirs(data_dir, exist_ok=True)
            db_path = os.path.join(data_dir, 'sui_indexer.db')
            
            # Set database URL with absolute path
            os.environ['DATABASE_URL'] = f'file:{db_path}'
            
            self.get_logger().info(f"Database path: {db_path}")
            self.get_logger().info(f"Database URL: {os.environ['DATABASE_URL']}")
            
            # Run setup
            self.setup_prisma()
            
        except Exception as e:
            self.get_logger().error(f'Error during initialization: {str(e)}')
            sys.exit(1)
        
    def setup_prisma(self):
        """Run Prisma generate and migrate."""
        try:
            # Generate Prisma client
            self.get_logger().info('Generating Prisma client...')
            result = subprocess.run(
                ['prisma', 'generate'], 
                cwd=self.prisma_path, 
                check=True,
                capture_output=True,
                text=True
            )
            self.get_logger().info(result.stdout)
            
            # Push the schema to the database
            self.get_logger().info('Pushing schema to database...')
            result = subprocess.run(
                ['prisma', 'db', 'push', '--accept-data-loss'], 
                cwd=self.prisma_path, 
                check=True,
                capture_output=True,
                text=True
            )
            self.get_logger().info(result.stdout)
            
            self.get_logger().info('Prisma setup completed successfully')
            
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Prisma command failed: {e.stderr}')
            sys.exit(1)
        except Exception as e:
            self.get_logger().error(f'Unexpected error during Prisma setup: {str(e)}')
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = PrismaSetupNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 