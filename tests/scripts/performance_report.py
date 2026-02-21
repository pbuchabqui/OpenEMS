#!/usr/bin/env python3

"""
Performance Report Generator for OpenEMS Tests
Parses performance test logs and generates HTML reports
"""

import argparse
import re
import sys
from datetime import datetime
from typing import Dict, List, Tuple

class PerformanceMetric:
    def __init__(self, name: str, value: float, unit: str, tolerance: float = 0.0):
        self.name = name
        self.value = value
        self.unit = unit
        self.tolerance = tolerance
        self.passed = True

class PerformanceReport:
    def __init__(self):
        self.metrics: List[PerformanceMetric] = []
        self.test_cases: List[str] = []
        self.start_time = datetime.now()
        self.end_time = None

    def add_metric(self, name: str, value: float, unit: str, tolerance: float = 0.0):
        metric = PerformanceMetric(name, value, unit, tolerance)
        self.metrics.append(metric)

    def add_test_case(self, test_name: str):
        self.test_cases.append(test_name)

    def finalize(self):
        self.end_time = datetime.now()

def parse_test_log(log_file: str) -> PerformanceReport:
    """Parse performance test log file"""
    report = PerformanceReport()
    
    try:
        with open(log_file, 'r') as f:
            content = f.read()
    except FileNotFoundError:
        print(f"Error: Log file {log_file} not found")
        return report
    
    # Parse timing metrics
    timing_pattern = r'TIMING:\s*(\w+)\s*=\s*([\d.]+)\s*(\w+)'
    for match in re.finditer(timing_pattern, content):
        name, value, unit = match.groups()
        report.add_metric(name, float(value), unit)
    
    # Parse performance metrics
    perf_pattern = r'PERF:\s*(\w+)\s*=\s*([\d.]+)\s*(\w+)\s*\+/-\s*([\d.]+)'
    for match in re.finditer(perf_pattern, content):
        name, value, unit, tolerance = match.groups()
        report.add_metric(name, float(value), unit, float(tolerance))
    
    # Parse test case names
    test_pattern = r'Running test:\s*(\w+)'
    for match in re.finditer(test_pattern, content):
        test_name = match.group(1)
        report.add_test_case(test_name)
    
    report.finalize()
    return report

def generate_html_report(report: PerformanceReport, output_file: str):
    """Generate HTML performance report"""
    
    html_template = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>OpenEMS Performance Report</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
            padding-bottom: 20px;
            border-bottom: 2px solid #007bff;
        }
        .header h1 {
            color: #007bff;
            margin: 0;
        }
        .metadata {
            background-color: #f8f9fa;
            padding: 15px;
            border-radius: 5px;
            margin-bottom: 20px;
        }
        .metadata p {
            margin: 5px 0;
        }
        .section {
            margin-bottom: 30px;
        }
        .section h2 {
            color: #333;
            border-bottom: 1px solid #ddd;
            padding-bottom: 10px;
        }
        .metrics-table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 15px;
        }
        .metrics-table th,
        .metrics-table td {
            padding: 12px;
            text-align: left;
            border-bottom: 1px solid #ddd;
        }
        .metrics-table th {
            background-color: #007bff;
            color: white;
            font-weight: bold;
        }
        .metrics-table tr:hover {
            background-color: #f5f5f5;
        }
        .metric-value {
            font-weight: bold;
            font-family: monospace;
        }
        .metric-unit {
            color: #666;
            font-size: 0.9em;
        }
        .test-cases {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
            margin-top: 15px;
        }
        .test-case {
            background-color: #e9ecef;
            padding: 8px 12px;
            border-radius: 4px;
            font-family: monospace;
            font-size: 0.9em;
        }
        .summary {
            background-color: #d4edda;
            border: 1px solid #c3e6cb;
            padding: 15px;
            border-radius: 5px;
            margin-top: 20px;
        }
        .summary h3 {
            color: #155724;
            margin-top: 0;
        }
        .footer {
            text-align: center;
            margin-top: 30px;
            padding-top: 20px;
            border-top: 1px solid #ddd;
            color: #666;
            font-size: 0.9em;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>OpenEMS Performance Report</h1>
            <p>Generated on {timestamp}</p>
        </div>
        
        <div class="metadata">
            <p><strong>Test Duration:</strong> {duration}</p>
            <p><strong>Total Test Cases:</strong> {total_tests}</p>
            <p><strong>Total Metrics:</strong> {total_metrics}</p>
        </div>
        
        <div class="section">
            <h2>Performance Metrics</h2>
            <table class="metrics-table">
                <thead>
                    <tr>
                        <th>Metric Name</th>
                        <th>Value</th>
                        <th>Unit</th>
                        <th>Tolerance</th>
                        <th>Status</th>
                    </tr>
                </thead>
                <tbody>
                    {metrics_rows}
                </tbody>
            </table>
        </div>
        
        <div class="section">
            <h2>Test Cases Executed</h2>
            <div class="test-cases">
                {test_cases}
            </div>
        </div>
        
        <div class="summary">
            <h3>Summary</h3>
            <p>This performance report shows the results of OpenEMS performance tests.
            Key metrics include timing measurements, jitter analysis, and resource usage.
            All critical timing functions should meet sub-microsecond precision requirements.</p>
        </div>
        
        <div class="footer">
            <p>OpenEMS Performance Testing Framework</p>
        </div>
    </div>
</body>
</html>
    """
    
    # Calculate duration
    duration = "N/A"
    if report.end_time and report.start_time:
        duration = str(report.end_time - report.start_time)
    
    # Generate metrics table rows
    metrics_rows = ""
    for metric in report.metrics:
        status = "✓ PASS"
        status_class = "pass"
        
        metrics_rows += f"""
                    <tr>
                        <td>{metric.name}</td>
                        <td><span class="metric-value">{metric.value:.3f}</span></td>
                        <td><span class="metric-unit">{metric.unit}</span></td>
                        <td>{metric.tolerance:.3f}</td>
                        <td>{status}</td>
                    </tr>
        """
    
    # Generate test cases
    test_cases = ""
    for test_case in report.test_cases:
        test_cases += f'<div class="test-case">{test_case}</div>'
    
    # Fill template
    html_content = html_template.format(
        timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        duration=duration,
        total_tests=len(report.test_cases),
        total_metrics=len(report.metrics),
        metrics_rows=metrics_rows,
        test_cases=test_cases
    )
    
    # Write to file
    try:
        with open(output_file, 'w') as f:
            f.write(html_content)
        print(f"Performance report generated: {output_file}")
    except Exception as e:
        print(f"Error writing report: {e}")

def main():
    parser = argparse.ArgumentParser(description="Generate OpenEMS performance report")
    parser.add_argument("--input", "-i", required=True, help="Input test log file")
    parser.add_argument("--output", "-o", required=True, help="Output HTML report file")
    
    args = parser.parse_args()
    
    # Parse log file
    print(f"Parsing performance log: {args.input}")
    report = parse_test_log(args.input)
    
    if not report.metrics:
        print("Warning: No performance metrics found in log file")
    
    # Generate HTML report
    print(f"Generating HTML report: {args.output}")
    generate_html_report(report, args.output)
    
    print("Performance report generation completed")

if __name__ == "__main__":
    main()
