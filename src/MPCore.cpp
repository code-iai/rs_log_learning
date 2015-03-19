/*
 * MPCore.cpp
 *
 *  Created on: Feb 17, 2015
 *      Author: andre
 */

#include "MPCore.h"

namespace rs_log_learn
{

MPCore::MPCore(ConfigParams params)
{
    this->configParams_ = params;
}

MPCore::~MPCore()
{
}

void MPCore::initialize()
{
    learnAS_ = LearnAnnotationStorage(this->configParams_.learningHost,
            this->configParams_.learningDB);
}

void MPCore::process(uima::CAS &tcas)
{
    initialize();

    outInfo("------------------------");
    outInfo("Learning Host = " << this->configParams_.learningHost);
    outInfo("Learning DB   = " << this->configParams_.learningDB);
    outInfo("Annotator in mode = " << this->configParams_.mode);
    outInfo("Annotator using algorithm = " << this->configParams_.algorithm);
    outInfo("------------------------");

    // decide strategy based on mode
    if(configParams_.mode.compare("train") == 0)
    {
        train(tcas);
    }
    else if(configParams_.mode.compare("learn") == 0)
    {
        learn(tcas);
    }
    else
    {
        outError("wrong mode. check pipeline configuration");
    }
}


void MPCore::loadDB(uima::CAS &tcas)
{
    // only load from learning db on first frame.
    // after this everything will be in memory for the pipeline run
    iai_rs::SceneCas cas(tcas);

    std::vector<iai_rs::Cluster> clusters;
    cas.getScene().identifiables.filter(clusters);

    // FIXME: still burning the first CAS this way
    // load data from db
    if (!learnDBloaded)
    {
        outInfo("loading learnDB");
        learnIdentifiables_ = learnAS_.extractLearnIdentifiables(tcas);
        learnDBloaded = true;
        outInfo("db loaded.");
    }
}
/*
 * Continuous learning
 */
void MPCore::learn(uima::CAS &tcas)
{
    iai_rs::SceneCas cas(tcas);

    std::vector<iai_rs::Cluster> clusters;
    cas.getScene().identifiables.filter(clusters);

    // load data from db
    loadDB(tcas);

    // only take the most recent learned data into account
    while(delete_next_turn != 0)
    {
        outInfo("deleting in this turn");
        learnIdentifiables_.pop_back();
        --delete_next_turn;
    }

    if(additionalsAvailable)
    {
        outInfo("adding additional learned dt idents");
        learnIdentifiables_.insert(learnIdentifiables_.end(), additionalDTIdentifiables_.begin(), additionalDTIdentifiables_.end());
        additionalDTIdentifiables_.clear();
        additionalsAvailable = false;
    }

    delete_next_turn = pushed_back_idents;
    pushed_back_idents = 0;
    // process clusters of the current CAS and match against loaded data
    // add learned data to reference identifiables after each run
    for (int i = 0; i < clusters.size(); ++i)
    {
        iai_rs::Learning lrn = iai_rs::create<iai_rs::Learning>(tcas);

        outInfo("creating queryident");
        MPIdentifiable queryIdentifiable = extractIdentifiableFromCluster(clusters[i]);
        MPIdentifiable resultIdentifiable(0);

        // switch based on learning algo config
        if(configParams_.algorithm.compare("knn") == 0)
        {
            NearestNeighborAlgorithm knn;
            resultIdentifiable = knn.process(learnIdentifiables_, queryIdentifiable);
            // only add the result, if the confidence is high enough to prevent bad data
            if(resultIdentifiable.getLearningAnnotation().getConfidence() > CONFIDENCE_THRESHOLD)
            {
                learnIdentifiables_.push_back(resultIdentifiable);
            }
        }
        else if(configParams_.algorithm.compare("dt") == 0)
        {
            DecisionTreeAlgorithm dt;
            resultIdentifiable = dt.process(learnIdentifiables_, queryIdentifiable);
            additionalDTIdentifiables_.push_back(resultIdentifiable);

            additionalsAvailable = true;
            ++pushed_back_idents;
        }
        else
        {
            outError("Algorithm " << configParams_.algorithm << " unknown!");
            return;
        }

        lrn.name.set(resultIdentifiable.getLearningAnnotation().getLearnedName());
        lrn.shape.set(resultIdentifiable.getLearningAnnotation().getShape());
        lrn.confidence.set(resultIdentifiable.getLearningAnnotation().getConfidence());
        outInfo("lrn data to append: " << lrn.name.get());
        clusters[i].annotations.append(lrn);
    }

    sceneNo++;
}

/*
 * Just learn from available data non continuously
 */
void MPCore::train(uima::CAS &tcas)
{
    iai_rs::SceneCas cas(tcas);

    std::vector<iai_rs::Cluster> clusters;
    cas.getScene().identifiables.filter(clusters);

    // load data from db
    loadDB(tcas);

    // process clusters of the current CAS and match against loaded data
    for (int i = 0; i < clusters.size(); ++i)
    {
        iai_rs::Learning lrn = iai_rs::create<iai_rs::Learning>(tcas);

        outInfo("creating queryident");
        MPIdentifiable queryIdentifiable = extractIdentifiableFromCluster(clusters[i]);
        MPIdentifiable resultIdentifiable(0);

        // switch based on learning algo config
        if(configParams_.algorithm.compare("knn") == 0)
        {
            NearestNeighborAlgorithm knn;
            resultIdentifiable = knn.process(learnIdentifiables_, queryIdentifiable);
        }
        else if(configParams_.algorithm.compare("dt") == 0)
        {
            DecisionTreeAlgorithm dt;
            resultIdentifiable = dt.process(learnIdentifiables_, queryIdentifiable);
        }
        else
        {
            outError("Algorithm " << configParams_.algorithm << " unknown!");
            return;
        }

        lrn.name.set(resultIdentifiable.getLearningAnnotation().getLearnedName());
        lrn.shape.set(resultIdentifiable.getLearningAnnotation().getShape());
        lrn.confidence.set(resultIdentifiable.getLearningAnnotation().getConfidence());
        outInfo("lrn data to append: " << lrn.name.get());
        clusters[i].annotations.append(lrn);
    }
    sceneNo++;
}

/**
 * Extract an MPIdentifiable structure from a given cluster
 */
MPIdentifiable MPCore::extractIdentifiableFromCluster(iai_rs::Cluster cluster)
{
    MPIdentifiable queryIdentifiable(0); // dummy timestamp. not needed

    std::vector<iai_rs::Geometry> iaiGeometry;
    std::vector<iai_rs::SemanticColor> iaiSemanticColor;
    cluster.annotations.filter(iaiGeometry);
    cluster.annotations.filter(iaiSemanticColor);

    if (iaiGeometry.empty())
    {
        outError("geometry empty");
    }
    else
    {
        queryIdentifiable.setGeometry(Geometry(iaiGeometry.at(0)));
    }
    if (iaiSemanticColor.empty())
    {
        outError("semanticColor empty");
    }
    else
    {
        queryIdentifiable.setSemColor(SemanticColor(iaiSemanticColor.at(0)));
    }

    return queryIdentifiable;
}

} /* namespace rs_log_learn */
