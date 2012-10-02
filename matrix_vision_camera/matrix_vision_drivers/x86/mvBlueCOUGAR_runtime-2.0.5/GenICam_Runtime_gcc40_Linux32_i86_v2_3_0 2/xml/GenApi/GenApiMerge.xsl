<?xml version="1.0" encoding="UTF-8"?>

<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform" xmlns:my1_0="http://www.genicam.org/GenApi/Version_1_0" xmlns:my1_1="http://www.genicam.org/GenApi/Version_1_1">
  
  <xsl:output method="xml"  version="1.0" encoding="utf-8" indent="yes"/>
    
  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->
  <!-- Some useful lists                                                -->
  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->


  <!-- The file where the nodes come from -->
  <!-- If you are using a style sheet processot you must set the file name accordingly -->
  <!-- GenApi uses the default to feed in the data via an internal stream -->
  <xsl:variable name="InjectFile" select="'http://InjectFile'" /> 

  <!-- Use this for debugging -->
  <!--<xsl:variable name="InjectFile" select="'C:\Projects\GenICam\xml\GenApi\GenApiTest\SchemaTestSuite_TestTicket873_Inject.xml'" />-->

  <!-- The nodes to be injected -->
  <xsl:variable name="InjectNodes" select="document($InjectFile)" />
  
  <!-- The Names of the nodes to be injected -->
  <xsl:variable name="InjectNames" select="$InjectNodes//*[not(name()='pVariable') and not(name()='EnumEntry') and not(name()='Constant') and not(name()='Expression')]/@Name" />

  <!-- The Names of the nodes in the target file -->
  <xsl:variable name="TargetNames" select="//*[not(name()='pVariable') and not(name()='EnumEntry')]/@Name" />

  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->
  <!-- Main template: matches RegisterDescription                       -->
  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->
  <xsl:template match="/my1_0:RegisterDescription|/my1_1:RegisterDescription">
    <xsl:copy>

      <!-- Copy the RegisterDescription's attributes -->
      <xsl:copy-of select="@*"/>

      <!-- Handle the target nodes -->
      <xsl:apply-templates select="*"/>

      <!-- Handle the injected nodes -->
      <xsl:apply-templates select="$InjectNodes/my1_0:RegisterDescription/*|$InjectNodes/my1_1:RegisterDescription/*" mode="Inject"/>

    </xsl:copy>
  </xsl:template>

  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->
  <!-- Handle Category nodes                                            -->
  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->

  <!-- Matches target Category nodes which want to be merged -->
  <xsl:template match="my1_0:Category|my1_1:Category" priority="1">
    <xsl:variable name="CategoryName" select="@Name" />
    <xsl:choose>

      <!-- Merge Categories which are also present in the injected file -->
      <xsl:when test="$CategoryName=$InjectNames">
        <xsl:copy>
          <!-- Copy Attributes -->
          <xsl:copy-of select="@*"/>

          <!-- Copy all target elements -->
          <xsl:for-each select="*">
            <xsl:copy-of select="."/>
          </xsl:for-each>

          <!-- If there is an injected Category with the same Name copy all pFeature entries except those also present in the target file -->
          <xsl:variable name="TargetFeatures" select="//my1_0:Category[$CategoryName=@Name]/my1_0:pFeature|//my1_1:Category[$CategoryName=@Name]/my1_1:pFeature" />
          <xsl:for-each select="$InjectNodes//my1_0:Category[$CategoryName=@Name]/my1_0:pFeature|$InjectNodes//my1_1:Category[$CategoryName=@Name]/my1_1:pFeature">
            <xsl:if test="not(.=$TargetFeatures)">
              <xsl:copy-of select="."/>
            </xsl:if>
          </xsl:for-each>

        </xsl:copy>
      </xsl:when>

      <!-- Copy Categories which are not present in the injected file -->
      <xsl:otherwise>
        <xsl:copy-of select="."/>
      </xsl:otherwise>
    </xsl:choose>

  </xsl:template>

  <!-- Match injected Categories -->
  <xsl:template match="my1_0:Category|my1_1:Category" mode="Inject" priority="1">
    
    <!-- Copy Category if it is not present in the target file -->
    <xsl:if test="not(@Name=$TargetNames)">
      <xsl:apply-templates select="." mode="CopyCategory"/>
    </xsl:if>
    
  </xsl:template>

  <!-- Copy everything from the category -->
  <xsl:template match="@*|node()|comment()" mode="CopyCategory">
     <!-- don't copy the MergePriority attribute in order to support schema v1.0 -->
     <xsl:if test="not( (count(. | ../@*) = count(../@*)) and name()='MergePriority' )">
        <xsl:copy>
           <xsl:apply-templates select="@*|node()|comment()" mode="CopyCategory"/>
        </xsl:copy>
     </xsl:if>
  </xsl:template>


  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->
  <!-- Handle Enumeration nodes                                         -->
  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->

  <!-- Matches target Enumeration nodes which want to be merged -->
  <xsl:template match="my1_0:Enumeration|my1_1:Enumeration" priority="1">
    <xsl:variable name="EnumerationName" select="@Name" />
    <xsl:choose>

      <!-- Merge Enumerations which are also present in the injected file -->
      <xsl:when test="$EnumerationName=$InjectNames">
      
        <xsl:variable name="InjectMergePriority" select="$InjectNodes//my1_0:Enumeration[(@Name=$EnumerationName)]/@MergePriority|
        $InjectNodes//my1_1:Enumeration[(@Name=$EnumerationName)]/@MergePriority" />
        <xsl:choose>
          
          <!-- If the injected node has MergePriority=-1 ==> Copy the target node -->
          <xsl:when test="$InjectMergePriority='-1'" >
            <xsl:copy>
              <xsl:apply-templates select="@*|node()"/>
            </xsl:copy>
          </xsl:when>
          
          <!-- If the injected node has MergePriority=+1 ==> Copy the injected node -->
          <xsl:when test="$InjectMergePriority='+1'" >
            <xsl:copy>
              <xsl:apply-templates select="@*|node()" mode="CopyEnumeration"/>
            </xsl:copy>
          </xsl:when>
          
          <xsl:otherwise>
            <xsl:copy>
              <!-- Copy Attributes -->
              <xsl:copy-of select="@*"/>
    
              <!-- Copy all target elements except EnumEntry entries also present in the injection file with MergePriority=+1 -->
              <xsl:variable name="InjectedEntries" select="$InjectNodes//my1_0:Enumeration[$EnumerationName=@Name]/my1_0:EnumEntry|
                                                           $InjectNodes//my1_1:Enumeration[$EnumerationName=@Name]/my1_1:EnumEntry" />
              <xsl:for-each select="*">
                <xsl:if test="not(./@Name=$InjectedEntries[@MergePriority='+1']/@Name) and not(name()='Value') and not(name()='pValue') and not(name()='pSelected')">
                  <xsl:copy-of select="."/>
                </xsl:if>
              </xsl:for-each>
    
              <!-- If there is an injected Enumeration with the same Name copy all EnumEntry entries not present in the target or with MergePriority=+1 -->
                <xsl:variable name="TargetEntries" select="//my1_0:Enumeration[$EnumerationName=@Name]/my1_0:EnumEntry|
                                                           //my1_1:Enumeration[$EnumerationName=@Name]/my1_1:EnumEntry" />
                <xsl:for-each select="$InjectNodes//my1_0:Enumeration[$EnumerationName=@Name]/my1_0:EnumEntry|
                                    $InjectNodes//my1_1:Enumeration[$EnumerationName=@Name]/my1_1:EnumEntry">
                <xsl:if test="not(./@MergePriority='-1') or not(./@Name=$TargetEntries/@Name)">
                    <xsl:apply-templates select="." mode="Inject"/>
                </xsl:if>
              </xsl:for-each>

              <!-- These nodes come after the EnumEntry entries -->
              <xsl:for-each select="*">
                <xsl:if test="(name()='Value') or (name()='pValue') or (name()='pSelected')">
                  <xsl:copy-of select="."/>
                </xsl:if>
              </xsl:for-each>
              
            </xsl:copy>
          </xsl:otherwise>

        </xsl:choose>
      </xsl:when>

      <!-- Copy Enumerations which are not present in the injected file -->
      <xsl:otherwise>
        <xsl:copy-of select="."/>
      </xsl:otherwise>
    </xsl:choose>

  </xsl:template>

  <!-- Match injected Enumerations -->
  <xsl:template match="my1_0:Enumeration|my1_1:Enumeration" mode="Inject" priority="1">
    
    <!-- Copy Category if it is not present in the target file -->
    <xsl:if test="not(@Name=$TargetNames)">
      <xsl:apply-templates select="." mode="CopyEnumeration"/>
    </xsl:if>
    
  </xsl:template>

  <!-- Copy everything from the Enumeration -->
  <xsl:template match="@*|node()|comment()" mode="CopyEnumeration">
     <!-- don't copy the MergePriority attribute in order to support schema v1.0 -->
     <xsl:if test="not( (count(. | ../@*) = count(../@*)) and name()='MergePriority' )">
        <xsl:copy>
           <xsl:apply-templates select="@*|node()|comment()" mode="CopyEnumeration"/>
        </xsl:copy>
     </xsl:if>
  </xsl:template>


  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->
  <!-- Handle non-Category nodes with a @Name attribute                 -->
  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->

  <!-- Handle Target Nodes -->
  <xsl:template match="my1_0:*[string-length(@Name)!=0]|my1_1:*[string-length(@Name)!=0]">
    <xsl:choose>

      <!-- If there is no node with the same Name in the InjectFile ==> Copy -->
      <xsl:when test="not(@Name=$InjectNames)">
        <xsl:copy>
          <xsl:apply-templates select="@*|node()"/>
        </xsl:copy>
      </xsl:when>

      <xsl:otherwise>
        <xsl:variable name="MyName" select="@Name" />
        <xsl:variable name="InjectMergePriority" select="$InjectNodes//*[(@Name=$MyName) and not(name()='pVariable') and not(name()='EnumEntry') and not(name()='Constant') and not(name()='Expression')]/@MergePriority" />
        <xsl:choose>
          
          <!-- If the injected node has MergePriority=-1 ==> Copy the target node -->
          <xsl:when test="$InjectMergePriority='-1'" >
            <xsl:copy>
              <xsl:apply-templates select="@*|node()"/>
            </xsl:copy>
          </xsl:when>

          <!-- If the injected node has InjectMergePriority=0 or no attribute set ==> Error -->
          <xsl:when test="$InjectMergePriority='0' or string-length($InjectMergePriority)=0" >
            <xsl:message terminate="yes">
              Error : There are two nodes with the same name '<xsl:value-of select="@Name"/>' but the injected node has MergePriority=0 or no MergePriority attribute
            </xsl:message>
          </xsl:when>

          <!-- If the injected node has MergePriority=+1 ==> don't copy -->

        </xsl:choose>
      </xsl:otherwise>

    </xsl:choose>
  </xsl:template>

  <!-- Handle Inject Nodes -->
  <xsl:template match="*[string-length(@Name)!=0]" mode="Inject">
    <xsl:choose>

      <!-- If there is no node with the same Name in the TargetFile ==> Copy -->
      <xsl:when test="not(@Name = $TargetNames)">
        <xsl:copy>
          <xsl:apply-templates select="@*|node()" mode="Inject"/>
        </xsl:copy>
      </xsl:when>

      <xsl:otherwise>

        <xsl:variable name="MyName" select="@Name" />
        <xsl:variable name="InjectMergePriority" select="$InjectNodes//*[(@Name=$MyName) and not(name()='pVariable') and not(name()='EnumEntry') and not(name()='Constant') and not(name()='Expression')]/@MergePriority" />

        <!-- If the inject node has priority +1 ==> Copy -->
        <xsl:if test="$InjectMergePriority='+1'" >
          <xsl:copy>
            <xsl:apply-templates select="@*|node()" mode="Inject"/>
          </xsl:copy>
        </xsl:if>

        <!-- No need to handle the error cases because they are handled on the target side -->
        
      </xsl:otherwise>

    </xsl:choose>
  </xsl:template>

  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->
  <!-- Handle all other nodes and attributes                            -->
  <!-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ -->

  <!-- Copy everything from the target file which has not yet been matched by the rules above -->
  <xsl:template match="@*|node()|comment()">
    <xsl:copy>
      <xsl:apply-templates select="@*|node()|comment()"/>
    </xsl:copy>
  </xsl:template>
  
  <!-- Copy everything from the injected file which has not yet been matched by the rules above -->
  <xsl:template match="@*|node()|comment()" mode="Inject">
    <!-- don't copy the MergePriority attribute in order to support schema v1.0 -->
    <xsl:if test="not( (count(. | ../@*) = count(../@*)) and name()='MergePriority' )">
      <xsl:copy>
        <xsl:apply-templates select="@*|node()|comment()" mode="Inject"/>
      </xsl:copy>
    </xsl:if>
  </xsl:template>

</xsl:stylesheet>
